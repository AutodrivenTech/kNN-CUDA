#include <pcl/common/io.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <gflags/gflags.h>

#include "knncuda.h"
#include "power.h"

#define K_DATA_MAX_VALUE 500

DEFINE_string(dataset_dir, "/tmp/same_queryset",
              "The directory path of dataset data");
DEFINE_string(queryset_dir, "/tmp/same_dataset",
              "The directory path of queryset data");
DEFINE_int32(sleep_time, 10, "the sleep time between each selection");
DEFINE_int32(to, 20, "to id");
DEFINE_int32(i, 100, "iteriation time.");

namespace fs = std::filesystem;
double static_power = 0.0;

/**
 * Check if a file exists
 * @return true if and only if the file exists, false else
 */
bool file_exists(const char *file) {
  struct stat buf;
  return (stat(file, &buf) == 0);
}

/**
 * Read data into program
 * @param data refence points
 * @param data_nb number of reference points
 * @param dir_name the path to the directory of data set
 * @param file_name the name of the file
 **/
bool read_data(float *data, int *data_nb, const std::string &dir_name,
               const std::string &file_name) {
  std::stringstream ss;
  fs::path dir(dir_name);
  fs::path file_path(file_name);
  fs::path full_path = dir / file_path;

  if (!file_exists(full_path.c_str())) {
    std::cerr << "File:" << full_path << " not exist." << std::endl;
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(full_path.c_str(), *cloud) == -1) {
    std::cerr << "Load pcd file failed!" << std::endl;
    return false;
  }

  size_t point_idx = 0;
  // std::cout << cloud->points.size() << std::endl;
  for (auto i = 0; i < cloud->points.size(); i++) {
    if (abs(cloud->points[i].x) < K_DATA_MAX_VALUE &&
        abs(cloud->points[i].y) < K_DATA_MAX_VALUE &&
        abs(cloud->points[i].z) < K_DATA_MAX_VALUE) {
      data[point_idx * 3] = cloud->points[i].x;
      data[point_idx * 3 + 1] = cloud->points[i].y;
      data[point_idx * 3 + 2] = cloud->points[i].z;
      point_idx++;
      (*data_nb)++;
    }
  }
  return true;
}

/**
 * Initializes randomly the reference and query points.
 *
 * @param ref        refence points
 * @param ref_nb     number of reference points
 * @param query      query points
 * @param query_nb   number of query points
 * @param dim        dimension of points
 */
void initialize_data(float *ref, int ref_nb, float *query, int query_nb,
                     int dim) {
  // Initialize random number generator
  srand(time(NULL));

  // Generate random reference points
  for (int i = 0; i < ref_nb * dim; ++i) {
    ref[i] = 10. * (float)(rand() / (double)RAND_MAX);
  }

  // Generate random query points
  for (int i = 0; i < query_nb * dim; ++i) {
    query[i] = 10. * (float)(rand() / (double)RAND_MAX);
  }
}

/**
 * Computes the Euclidean distance between a reference point and a query point.
 *
 * @param ref          refence points
 * @param ref_nb       number of reference points
 * @param query        query points
 * @param query_nb     number of query points
 * @param dim          dimension of points
 * @param ref_index    index to the reference point to consider
 * @param query_index  index to the query point to consider
 * @return computed distance
 */
float compute_distance(const float *ref, int ref_nb, const float *query,
                       int query_nb, int dim, int ref_index, int query_index) {
  float sum = 0.f;
  for (int d = 0; d < dim; ++d) {
    const float diff =
        ref[d * ref_nb + ref_index] - query[d * query_nb + query_index];
    sum += diff * diff;
  }
  return sqrtf(sum);
}

/**
 * Gathers at the beginning of the `dist` array the k smallest values and their
 * respective index (in the initial array) in the `index` array. After this
 * call, only the k-smallest distances are available. All other distances might
 * be lost.
 *
 * Since we only need to locate the k smallest distances, sorting the entire
 * array would not be very efficient if k is relatively small. Instead, we
 * perform a simple insertion sort by eventually inserting a given distance in
 * the first k values.
 *
 * @param dist    array containing the `length` distances
 * @param index   array containing the index of the k smallest distances
 * @param length  total number of distances
 * @param k       number of smallest distances to locate
 */
void modified_insertion_sort(float *dist, int *index, int length, int k) {
  // Initialise the first index
  index[0] = 0;

  // Go through all points
  for (int i = 1; i < length; ++i) {
    // Store current distance and associated index
    float curr_dist = dist[i];
    int curr_index = i;

    // Skip the current value if its index is >= k and if it's higher the k-th
    // slready sorted mallest value
    if (i >= k && curr_dist >= dist[k - 1]) {
      continue;
    }

    // Shift values (and indexes) higher that the current distance to the right
    int j = std::min(i, k - 1);
    while (j > 0 && dist[j - 1] > curr_dist) {
      dist[j] = dist[j - 1];
      index[j] = index[j - 1];
      --j;
    }

    // Write the current distance and index at their position
    dist[j] = curr_dist;
    index[j] = curr_index;
  }
}

/*
 * For each input query point, locates the k-NN (indexes and distances) among
 * the reference points.
 *
 * @param ref        refence points
 * @param ref_nb     number of reference points
 * @param query      query points
 * @param query_nb   number of query points
 * @param dim        dimension of points
 * @param k          number of neighbors to consider
 * @param knn_dist   output array containing the query_nb x k distances
 * @param knn_index  output array containing the query_nb x k indexes
 */
bool knn_c(const float *ref, int ref_nb, const float *query, int query_nb,
           int dim, int k, float *knn_dist, int *knn_index) {
  // Allocate local array to store all the distances / indexes for a given query
  // point
  float *dist = (float *)malloc(ref_nb * sizeof(float));
  int *index = (int *)malloc(ref_nb * sizeof(int));

  // Allocation checks
  if (!dist || !index) {
    printf("Memory allocation error\n");
    free(dist);
    free(index);
    return false;
  }

  // Process one query point at the time
  for (int i = 0; i < query_nb; ++i) {
    // Compute all distances / indexes
    for (int j = 0; j < ref_nb; ++j) {
      dist[j] = compute_distance(ref, ref_nb, query, query_nb, dim, j, i);
      index[j] = j;
    }

    // Sort distances / indexes
    modified_insertion_sort(dist, index, ref_nb, k);

    // Copy k smallest distances and their associated index
    for (int j = 0; j < k; ++j) {
      knn_dist[j * query_nb + i] = dist[j];
      knn_index[j * query_nb + i] = index[j];
    }
  }

  // Memory clean-up
  free(dist);
  free(index);

  return true;
}

/**
 * Test an input k-NN function implementation by verifying that its output
 * results (distances and corresponding indexes) are similar to the expected
 * results (ground truth).
 *
 * Since the k-NN computation might end-up in slightly different results
 * compared to the expected one depending on the considered implementation,
 * the verification consists in making sure that the accuracy is high enough.
 *
 * The tested function is ran several times in order to have a better estimate
 * of the processing time.
 *
 * @param ref            reference points
 * @param ref_nb         number of reference points
 * @param query          query points
 * @param query_nb       number of query points
 * @param dim            dimension of reference and query points
 * @param k              number of neighbors to consider
 * @param gt_knn_dist    ground truth distances
 * @param gt_knn_index   ground truth indexes
 * @param test_knn_dist  pre allocated knn dist points
 * @param test_knn_index pre-allocated knn index points
 * @param knn            function to test
 * @param name           name of the function to test (for display purpose)
 * @param nb_iterations  number of iterations
 * @param data_filename      file_name of output data
 * return false in case of problem, true otherwise
 */
bool test(const float *ref, int ref_nb, const float *query, int query_nb,
          int dim, int k, float *gt_knn_dist, int *gt_knn_index,
          float *test_knn_dist, int *test_knn_index,
          bool (*knn)(const float *, int, const float *, int, int, int, float *,
                      int *),
          const char *name, int nb_iterations, const char *data_filename) {
  // Parameters
  const float precision = 0.001f;     // distance error max
  const float min_accuracy = 0.999f;  // percentage of correct values required

  // Display k-NN function name
  printf("- %-17s : ", name);

  //   printf("allocation size is %ld\n", query_nb * k * sizeof(float));
  //   // Allocate memory for computed k-NN neighbors
  //   float *test_knn_dist = (float *)malloc(query_nb * k * sizeof(float));
  //   int *test_knn_index = (int *)malloc(query_nb * k * sizeof(int));

  //   Allocation check
  if (!test_knn_dist || !test_knn_index) {
    printf("ALLOCATION ERROR\n");
    // free(test_knn_dist);
    // free(test_knn_index);
    return false;
  }
  double total_power = 0.0;
  double usage_power = 0.0;

  nvml_monitor_start();
  // Start timer
  struct timeval tic;
  gettimeofday(&tic, NULL);

  // Compute k-NN several times
  for (int i = 0; i < nb_iterations; ++i) {
    if (!knn(ref, ref_nb, query, query_nb, dim, k, test_knn_dist,
             test_knn_index)) {
      //   free(test_knn_dist);
      //   free(test_knn_index);
      return false;
    }
  }

  // Stop timer
  struct timeval toc;
  gettimeofday(&toc, NULL);
  nvml_monitor_stop();

  total_power = integral_power_consuming();
  usage_power = total_power - static_power;

  // Elapsed time in ms
  double elapsed_time = toc.tv_sec - tic.tv_sec;
  elapsed_time += (toc.tv_usec - tic.tv_usec) / 1000000.;

  // Verify both precisions and indexes of the k-NN values
  int nb_correct_precisions = 0;
  int nb_correct_indexes = 0;
  for (int i = 0; i < query_nb * k; ++i) {
    if (fabs(test_knn_dist[i] - gt_knn_dist[i]) <= precision) {
      nb_correct_precisions++;
    }
    if (test_knn_index[i] == gt_knn_index[i]) {
      nb_correct_indexes++;
    }
  }

  // Compute accuracy
  float precision_accuracy = nb_correct_precisions / ((float)query_nb * k);
  float index_accuracy = nb_correct_indexes / ((float)query_nb * k);

  // Display report
  if (precision_accuracy >= min_accuracy && index_accuracy >= min_accuracy) {
    // printf("Precision_accuray: %f")
    //!! TODO: output precision
    // printf("usage power is %3.5f\n", usage_power);
    printf("precision is %0.5f and index accuracy is %0.5f\n",
           precision_accuracy, index_accuracy);
    printf("PASSED in %8.5f seconds (averaged over %3d iterations)\n",
           elapsed_time / nb_iterations, nb_iterations);
  } else {
    printf("FAILED\n");
  }

  printf("Write result to data file....\n");

  std::stringstream result;
  result << ref_nb << "\t" << query_nb << "\t" << k << "\t"
         << elapsed_time / nb_iterations << "\t" << static_power << "\t"
         << usage_power << '\n';
  std::cout << result.str();

  std::ofstream file(data_filename, std::ios_base::app);
  if (file.is_open()) {
    file << result.str();
  }

  // Free memory
  //   free(test_knn_dist);
  //   free(test_knn_index);

  return true;
}

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const int k = 5;
  const int dim = 3;
  // start power monitor
  nvml_api_init();

  sleep(3);
  printf("start monitoring static power\n");
  nvml_monitor_start();
  sleep(30);
  nvml_monitor_stop();
  static_power = integral_power_consuming();
  printf("static power is %4.5f w\n", static_power);
  printf("\n");

  // same queryset
  float *query = (float *)malloc(800000 * sizeof(float));
  float *knn_dist = (float *)malloc(800000 * sizeof(float));
  int *knn_index = (int *)malloc(800000 * sizeof(int));
  float *ref = (float *)malloc(800000 * sizeof(float));
  float *test_knn_dist = (float *)malloc(1800000 * sizeof(float));
  int *test_knn_index = (int *)malloc(800000 * sizeof(int));

SAME_QUERY_SET_CUDA_GLOABL : {
  std::cout << "same query set with cuda global" << std::endl;
  if (file_exists("same_queryset_cuda_global.txt")) {
    std::remove("same_queryset_cuda_global.txt");
  }

  for (int i = 0; i < FLAGS_to; i++) {
    int query_nb = 0;
    if (!read_data(query, &query_nb, FLAGS_dataset_dir,
                   "same_queryset_queryset.pcd")) {
      std::cerr << "open file"
                << "same_queryset_queryset.pcd"
                << " failed" << std::endl;
      return 0;
    }

    int ref_nb = 0;
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << i + 1 << "_dataset.pcd";
    std::string file_name = ss.str();
    std::cout << "dataset file is " << file_name << std::endl;
    if (!read_data(ref, &ref_nb, FLAGS_dataset_dir, file_name)) {
      std::cerr << "open file" << file_name << " failed" << std::endl;
      return 0;
    }

    printf("Ground truth computation in progress...\n\n");
    if (!knn_c(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index)) {
      // free(ref);
      free(query);
      free(knn_dist);
      free(knn_index);
      return EXIT_FAILURE;
    }

    //   printf("TESTS\n");
    auto test_res =
        test(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index,
             test_knn_dist, test_knn_index, knn_cuda_global, "knn_cuda_global",
             FLAGS_i, "same_queryset_cuda_global.txt");
    if (!test_res) {
      sleep(FLAGS_sleep_time);
      goto SAME_QUERY_SET_CUDA_TEXTURE;
    }
    //   //   memset((void*)query, 0, sizeof(float)*80000);
    //   free(query);
    //   free(ref);
    //   free(knn_dist);
    //   free(knn_index);
    std::cout << "sleep for " << std::to_string(FLAGS_sleep_time) << " s."
              << std::endl;
    sleep(FLAGS_sleep_time);
  }
}

SAME_QUERY_SET_CUDA_TEXTURE : {
  std::cout << "same query set with cuda texture" << std::endl;
  if (file_exists("same_queryset_cuda_texture.txt")) {
    std::remove("same_queryset_cuda_texture.txt");
  }

  for (int i = 0; i < FLAGS_to; i++) {
    int query_nb = 0;
    if (!read_data(query, &query_nb, FLAGS_dataset_dir,
                   "same_queryset_queryset.pcd")) {
      std::cerr << "open file"
                << "same_queryset_queryset.pcd"
                << " failed" << std::endl;
      return 0;
    }

    int ref_nb = 0;
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << i + 1 << "_dataset.pcd";
    std::string file_name = ss.str();
    std::cout << "dataset file is " << file_name << std::endl;
    if (!read_data(ref, &ref_nb, FLAGS_dataset_dir, file_name)) {
      std::cerr << "open file" << file_name << " failed" << std::endl;
      return 0;
    }

    printf("Ground truth computation in progress...\n\n");
    if (!knn_c(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index)) {
      // free(ref);
      free(query);
      free(knn_dist);
      free(knn_index);
      return EXIT_FAILURE;
    }

    //   printf("TESTS\n");
    auto test_res =
        test(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index,
             test_knn_dist, test_knn_index, knn_cuda_texture,
             "knn_cuda_texture", FLAGS_i, "same_queryset_cuda_texture.txt");
    if (!test_res) {
      sleep(FLAGS_sleep_time);
      goto SAME_DATA_SET_CUDA_GLOBAL;
    }
    //   //   memset((void*)query, 0, sizeof(float)*80000);
    //   free(query);
    //   free(ref);
    //   free(knn_dist);
    //   free(knn_index);
    std::cout << "sleep for " << std::to_string(FLAGS_sleep_time) << " s."
              << std::endl;
    sleep(FLAGS_sleep_time);
  }
}

SAME_DATA_SET_CUDA_GLOBAL : {
  std::cout << "same data set with cuda global" << std::endl;
  if (file_exists("same_dataset_cuda_global.txt")) {
    std::remove("same_dataset_cuda_global.txt");
  }

  for (int i = 0; i < FLAGS_to; i++) {
    int ref_nb = 0;
    if (!read_data(ref, &ref_nb, FLAGS_queryset_dir,
                   "same_dataset_dataset.pcd")) {
      std::cerr << "open file"
                << "same_dataset_dataset.pcd"
                << " failed" << std::endl;
      return 0;
    }

    int query_nb = 0;
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << i + 1 << "_queryset.pcd";
    std::string file_name = ss.str();
    std::cout << "dataset file is " << file_name << std::endl;
    if (!read_data(query, &query_nb, FLAGS_queryset_dir, file_name)) {
      std::cerr << "open file" << file_name << " failed" << std::endl;
      return 0;
    }

    printf("Ground truth computation in progress...\n\n");
    if (!knn_c(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index)) {
      // free(ref);
      free(query);
      free(knn_dist);
      free(knn_index);
      return EXIT_FAILURE;
    }

    //   printf("TESTS\n");
    auto test_res =
        test(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index,
             test_knn_dist, test_knn_index, knn_cuda_global, "knn_cuda_gloabl",
             FLAGS_i, "same_dataset_cuda_global.txt");
    if (!test_res) {
      sleep(FLAGS_sleep_time);
      goto SAME_DATA_SET_CUDA_TEXTURE;
    }
    //   //   memset((void*)query, 0, sizeof(float)*80000);
    //   free(query);
    //   free(ref);
    //   free(knn_dist);
    //   free(knn_index);
    std::cout << "sleep for " << std::to_string(FLAGS_sleep_time) << " s."
              << std::endl;
    sleep(FLAGS_sleep_time);
  }
}

SAME_DATA_SET_CUDA_TEXTURE : {
  std::cout << "same data set with cuda texture" << std::endl;
  if (file_exists("same_dataset_cuda_texture.txt")) {
    std::remove("same_dataset_cuda_texture.txt");
  }

  for (int i = 0; i < FLAGS_to; i++) {
    int ref_nb = 0;
    if (!read_data(ref, &ref_nb, FLAGS_queryset_dir,
                   "same_dataset_dataset.pcd")) {
      std::cerr << "open file"
                << "same_dataset_dataset.pcd"
                << " failed" << std::endl;
      return 0;
    }

    int query_nb = 0;
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << i + 1 << "_queryset.pcd";
    std::string file_name = ss.str();
    std::cout << "dataset file is " << file_name << std::endl;
    if (!read_data(query, &query_nb, FLAGS_queryset_dir, file_name)) {
      std::cerr << "open file" << file_name << " failed" << std::endl;
      return 0;
    }

    printf("Ground truth computation in progress...\n\n");
    if (!knn_c(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index)) {
      // free(ref);
      free(query);
      free(knn_dist);
      free(knn_index);
      return EXIT_FAILURE;
    }

    //   printf("TESTS\n");
    auto test_res =
        test(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index,
             test_knn_dist, test_knn_index, knn_cuda_texture,
             "knn_cuda_texture", FLAGS_i, "same_dataset_cuda_texture.txt");
    if (!test_res) {
      sleep(FLAGS_sleep_time);
      goto FREE_SCOPE;
    }
    //   //   memset((void*)query, 0, sizeof(float)*80000);
    //   free(query);
    //   free(ref);
    //   free(knn_dist);
    //   free(knn_index);
    std::cout << "sleep for " << std::to_string(FLAGS_sleep_time) << " s."
              << std::endl;
    sleep(FLAGS_sleep_time);
  }
}

  //   test(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index,
  //        &knn_cuda_texture, "knn_cuda_texture", 500);

  //   test(ref, ref_nb, query, query_nb, dim, k, knn_dist, knn_index,
  //   &knn_cublas,
  //        "knn_cublas", 500);

  // close power monitor
  //   nvml_api_close();
FREE_SCOPE:
  free(ref);
  free(query);
  free(knn_dist);
  free(knn_index);
  free(test_knn_dist);
  free(test_knn_index);
  return 0;
}