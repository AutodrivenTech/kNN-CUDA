/*
* EGBDS ON FPGA for dataset buffer
*/
#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <cstdlib>// Header file needed to use rand
#include <ctime> // Header file needed to use time
#include<fstream>

#define USE_SDX
#define USE_KDTREE_ON
#define DEBUG_MODE
//#define MEASURE_POWER
//#define TEST_MODE

#ifdef MEASURE_POWER
#include <time.h>
#include <fcntl.h>
#include "energy_meter.h"
#endif

#ifdef USE_SDX
#include "sds_lib.h"
#endif

#include "GBDS_extention.h"


#undef __ARM_NEON__
#undef __ARM_NEON

//#ifdef USE_KDTREE_ON
#include <nanoflann_pcl.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
//#endif

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#define __ARM_NEON__
#define __ARM_NEON

#ifdef TEST_MODE
#else
int main(int argc, char* argv[]) {


#ifdef MEASURE_POWER
	remove("TEST_EGBDS_PL_energy_meter.txt");
	struct timespec res;
	struct timespec dif;
	struct em_t em1, em2, em3;	//energe meter 1,2,3
	struct energy_sample* sample1;
	int sample_rate = 2;	//sample_rate = 2ms
	int debug_output_file = 1;
	double record_power_time[6];
	sample1 = energy_meter_init(sample_rate, debug_output_file);  // sample rate in miliseconds
	energy_meter_start(sample1);  // starts sampling thread

								  //obtain the absolute time of read power..
								  //	dif=diff(sample->start_time, sample->res[sample->now]);
								  //	fprintf(debugf,"%lf  ", (double)dif.tv_sec+ (double)dif.tv_nsec/1000000000.0);
#endif

	My_OutSplitInfo out_split_info_hw;	//min_x,max_x...min_z,max_z,split_array_x,y,z,split_unit_x,y,z
	My_OutSplitInfo out_split_info_sw;	//min_x,max_x...min_z,max_z,split_array_x,y,z,split_unit_x,y,z

										////************************************process he input and preparation work
	std::string input_file_path;
	std::string same_dataset_name = "same_dataset_dataset.pcd";
	std::string same_queryset_name = "same_queryset_queryset.pcd";
	float split_precise = 1;
	int max_points_num = k_data_set_size;
	int K = 5;
	int user_define_query_size = k_query_set_size;
	int loop_test_time;
	bool test_same_dataset_flag = 0;
#ifdef USE_SDX
	if (argc > 1)
	{
		input_file_path = argv[1];
		std::cout << "the input path is: " << input_file_path << std::endl;

	}
	else
	{
		std::cout << "error number of arguments.... " << (argc - 1) << std::endl << " You ought to give more than 2 arguments indicating the search path, print_info, max_data_number!" << std::endl;
		std::cout << "example: ./XXX.ELF filepath split_precise K loop_test_time query_number" << std::endl << std::endl;
		return -1;
	}
	if (argc > 2)
	{
		if (atof(argv[2]) > 0)
		{
			split_precise = atof(argv[2]);
		}
		else std::cout << "please enter a positive split_precision. default is 1" << std::endl;
	}
	if (argc > 3)
	{
		K = atoi(argv[3]);
		if (K > 10 || K < 1)
		{
			std::cout << "WRONG K NUMBER!" << std::endl;
			std::cout << "example: ./XXX.ELF filepath split_grid_number K query_number" << std::endl << "press any key to exit..." << std::endl;
			return -1;
		}
	}
	if (argc > 4)
	{
		loop_test_time = atoi(argv[4]);
	}
	if (argc > 5)
	{
		test_same_dataset_flag = atoi(argv[5]);
	}
#else
	input_file_path = "C:\\KITTI_TEST\\test_dir\\004_dataset.pcd";
	split_precise = 1;

	loop_test_time = 1;
    // 两种数据集
	test_same_dataset_flag = 1;	//0 for same sataset, 1 for same queryset
	user_define_query_size = k_query_set_size;

#endif

	if(test_same_dataset_flag)
		remove("TEST_EGBDS_SAME_DATASET.txt");
	else
		remove("TEST_EGBDS_SAME_QUERYSET.txt");

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	int file_name_min_index = 1;
	int file_name_max_index = 20;

	//if only one loop, then under debug mode

	std::string input_name_copy = input_file_path;
	int last_slash_index = input_name_copy.find_last_of("/\\");
	std::string dir_name = input_name_copy.substr(0, last_slash_index + 1);
	std::string input_file_name = input_name_copy.substr(last_slash_index + 1, 3);
	int input_file_index = atoi(input_file_name.c_str());

	int last_identifier_index = input_name_copy.find_last_of("_");
	std::string corner_surf_name = input_name_copy.erase(0, last_identifier_index + 1);

	if (loop_test_time == 1)
	{

		file_name_max_index = input_file_index;
		file_name_min_index = input_file_index;
	}

	for (int dataset_index = file_name_min_index; dataset_index <= file_name_max_index; dataset_index++)
	{


		std::string dataset_name;
		std::string query_name;
		std::stringstream ss;
		ss << std::setw(3) << std::setfill('0') << dataset_index << "_dataset.pcd";
		ss >> dataset_name;//or res = ss.str();
						   //if only one loop, then under debug mode
		std::string read_cloud_file_path;
		if (test_same_dataset_flag)
		{
			read_cloud_file_path = dir_name + same_dataset_name;
		}
		else
		{
			read_cloud_file_path = dir_name + dataset_name;
		}

		std::string read_query_file_path;
		std::stringstream ss_query;
		//ss_query << std::setw(3) << std::setfill('0') << dataset_index << "_query_" << corner_surf_name;
		ss_query << std::setw(3) << std::setfill('0') << dataset_index << "_queryset.pcd";
		ss_query >> query_name;//or res = ss.str();
		if (test_same_dataset_flag)
		{
			read_query_file_path = dir_name + query_name;
		}
		else
		{
			read_query_file_path = dir_name + same_queryset_name;
		}

		std::cout << "dataset path is : " << read_cloud_file_path << std::endl;
		std::cout << "query_set path is : " << read_query_file_path << std::endl;
		//////////////////////////////////////////////////////////////////////////////////////////////////////////


		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr query_cloud(new pcl::PointCloud<pcl::PointXYZI>);


		std::string file_type = read_cloud_file_path.substr(read_cloud_file_path.rfind(".") + 1);  //get file name
		std::transform(file_type.begin(), file_type.end(), file_type.begin(), ::tolower);
		if (file_type == "ply")
		{
			if (pcl::io::loadPLYFile<pcl::PointXYZI>(read_cloud_file_path, *cloud) == -1) //* load the file
			{
				PCL_ERROR("read file fail! \n");
				system("PAUSE");
				return (-1);
			}
		}
		else if (file_type == "pcd")
		{
			if (pcl::io::loadPCDFile<pcl::PointXYZI>(read_cloud_file_path, *cloud) == -1) //* load the file
			{
				PCL_ERROR("read file fail! \n");
				system("PAUSE");
				return (-1);
			}
			if (pcl::io::loadPCDFile<pcl::PointXYZI>(read_query_file_path, *query_cloud) == -1) //* load the file
			{
				PCL_ERROR("read query_set file fail! \n");
				system("PAUSE");
				return (-1);
			}
		}
		else
		{
			std::cout << "error point cloud file type. should be .ply or .pcd" << std::endl;
			system("PAUSE");
			return (-1);
		}


#ifdef USE_SDX
		My_Points* input_data_set = (My_Points*)sds_alloc(k_data_set_size * 3 * sizeof(type_point));
		type_point(*query_nearest_distance)[k_nearest_number_max] = (type_point(*)[k_nearest_number_max])sds_alloc(k_query_set_size * k_nearest_number_max * sizeof(type_point));
		int(*query_nearest_index)[k_nearest_number_max] = (int(*)[k_nearest_number_max])sds_alloc(k_query_set_size * k_nearest_number_max * sizeof(int));
		My_Points* my_point_sel = (My_Points*)sds_alloc(1 * 3 * sizeof(type_point));
		My_Points* query_set = (My_Points*)sds_alloc(k_query_set_size * 3 * sizeof(type_point));
		int* original_queryset_index = (int*)sds_alloc(k_query_set_size * sizeof(int));
#else
		My_Points input_data_set[k_data_set_size];
		type_point query_nearest_distance[k_query_set_size][k_nearest_number_max];
		int query_nearest_index[k_query_set_size][k_nearest_number_max];
		My_Points my_point_sel[1];
		My_Points query_set[k_query_set_size];
		int original_queryset_index[k_query_set_size];
#endif






		////************************************some parameter and prepare datast and query set
		My_MaxMin data_set_max_min;
		unsigned int cloud_points_size = 0;

		//transform point cloud to array; GBDS input format
		//remove nul points and overflow points
		for (unsigned int i = 0; i < cloud->points.size(); i++)
		{
			if (abs(cloud->points[i].x) < k_data_max_value_abs && abs(cloud->points[i].y) < k_data_max_value_abs && abs(cloud->points[i].z) < k_data_max_value_abs)
			{
				input_data_set[cloud_points_size].x = cloud->points[i].x;
				input_data_set[cloud_points_size].y = cloud->points[i].y;
				input_data_set[cloud_points_size].z = cloud->points[i].z;
				cloud_points_size++;
				if (cloud_points_size == (max_points_num))
				{
					std::cout << "cloud->points.size(): " << cloud->points.size() << std::endl << "max_data_set_size: " << max_points_num << std::endl;
					std::cout << "error, dataset size is bigger than array size. " << std::endl << "press any key to exit..." << std::endl;
					break;
					/*getchar();
					return -1;*/
				}
			}
		}
		

		std::cout << "cloud->points.size(): " << cloud_points_size << std::endl;
		std::cout << "over range data number: " << cloud->points.size() - cloud_points_size << std::endl;
		
		data_set_max_min = get_min_max(input_data_set, cloud_points_size);
		std::cout << "xmax_min: " << data_set_max_min.xmin << " " << data_set_max_min.xmax << std::endl;
		std::cout << "ymax_min: " << data_set_max_min.ymin << " " << data_set_max_min.ymax << std::endl;
		std::cout << "zmax_min: " << data_set_max_min.zmin << " " << data_set_max_min.zmax << std::endl;

		int query_points_size = 0;
		for (unsigned int i = 0; i < query_cloud->points.size(); i++)
		{
			if (abs(query_cloud->points[i].x) < k_data_max_value_abs && abs(query_cloud->points[i].y) < k_data_max_value_abs && abs(query_cloud->points[i].z) < k_data_max_value_abs)
			{
				query_set[query_points_size].x = query_cloud->points[i].x;
				query_set[query_points_size].y = query_cloud->points[i].y;
				query_set[query_points_size].z = query_cloud->points[i].z;
				query_points_size++;
				if (query_points_size == (user_define_query_size))
				{
					std::cout << "query_cloud->points.size(): " << query_cloud->points.size() << std::endl << "user_define_query_size: " << user_define_query_size << std::endl;
					std::cout << "error, user_define_query_size is bigger than array size. " << std::endl << "press any key to exit..." << std::endl;
					break;
					/*getchar();
					return -1;*/
				}
			}
		}

		std::cout << "query_points_size: " << query_points_size << std::endl;
		std::cout << "over range data number: " << query_cloud->points.size() - query_points_size << std::endl;



		///////////*****************************one query hw test	******************
		if (loop_test_time == 1)
		{
			std::cout << std::endl << std::endl << "Test software one query: " << std::endl << std::endl;
			//get the simulate input data
			{
				int random_select = 10;
				my_point_sel[0].x = input_data_set[random_select].x;
				my_point_sel[0].y = input_data_set[random_select].y;
				my_point_sel[0].z = input_data_set[random_select].z;
			}

			std::cout << "Before software ExGBDSIPCore_FULL_hw: " << std::endl;
			ExGBDSIPCore_FULL_sw(input_data_set, cloud_points_size, my_point_sel, 1, K, split_precise,
				query_nearest_index, query_nearest_distance, original_queryset_index, out_split_info_sw);

			std::cout << "query point: " << my_point_sel[0].x << " " << my_point_sel[0].y << " " << my_point_sel[0].z << std::endl;
			std::cout << "software nearest point: " << input_data_set[query_nearest_index[0][0]].x << " " << input_data_set[query_nearest_index[0][0]].y << " " << input_data_set[query_nearest_index[0][0]].z << std::endl;
			for (int i = 0; i < K; i++)
			{
				std::cout << i << " distance: " << query_nearest_distance[0][i] << " with index: " << query_nearest_index[0][i] << std::endl;
			}
			std::cout << std::endl;


			std::cout << "Before hardware ExGBDSIPCore_FULL_hw: " << std::endl;
			ExGBDSIPCore_FULL_hw(input_data_set, cloud_points_size, my_point_sel, 1, K, split_precise,
				query_nearest_index, query_nearest_distance, original_queryset_index, out_split_info_hw);
	
			std::cout << "hardware nearest point: " << input_data_set[query_nearest_index[0][0]].x << " " << input_data_set[query_nearest_index[0][0]].y << " " << input_data_set[query_nearest_index[0][0]].z << std::endl;
			for (int i = 0; i < K; i++)
			{
				std::cout << i << " distance: " << query_nearest_distance[0][i] << " with index: " << query_nearest_index[0][i] << std::endl;
			}
			std::cout << std::endl;
			std::cout << std::endl;

			//////////////////////////////kdtree for one query
			{
				//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
				nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeFromMap_test;
				kdtreeFromMap_test.setInputCloud(cloud);

				//my_point_sel = query_set[i];
				pcl::PointXYZI pointSel;
				pointSel.x = my_point_sel[0].x;
				pointSel.y = my_point_sel[0].y;
				pointSel.z = my_point_sel[0].z;
				std::vector<int> pointSearchInd;
				std::vector<float> pointSearchSqDis;
				pointSearchInd.resize(K);
				pointSearchSqDis.resize(K);
				kdtreeFromMap_test.nearestKSearch(pointSel, K, pointSearchInd, pointSearchSqDis);


				std::cout << "kdtree nearest point: " << input_data_set[pointSearchInd[0]].x << " " << input_data_set[pointSearchInd[0]].y << " " << input_data_set[pointSearchInd[0]].z << std::endl;
				for (int i = 0; i < K; i++)
				{
					std::cout << i << " distance: " << pointSearchSqDis[i] << " with index: " << pointSearchInd[i] << std::endl;
				}
				std::cout << std::endl;
			}

		}


		//repeat 100 times for average number
		if (loop_test_time > 1)
			loop_test_time = loop_test_time - 1;
		for (int j = 0; j < loop_test_time; j++)
		{


			///////////*********************test queryset with hw and sw
			int result_index_sw[k_query_set_size];
			float result_distance_sw[k_query_set_size];
			int result_index_hw[k_query_set_size];
			float result_distance_hw[k_query_set_size];

			std::cout << std::endl << std::endl << "Test Multi-queries: " << std::endl;


#ifdef MEASURE_POWER
			energy_meter_read(sample1, &em1);	 // take start reading
			record_power_time[0] = em1.current_time_stamp;
#endif

			//test software time
			ExGBDSIPCore_FULL_sw(input_data_set, cloud_points_size, query_set, query_points_size, K, split_precise,
				query_nearest_index, query_nearest_distance, original_queryset_index, out_split_info_sw);

			for (int i = 0; i < query_points_size; i++)
			{
				result_index_sw[i] = query_nearest_index[original_queryset_index[i]][0];
				result_distance_sw[i] = query_nearest_distance[original_queryset_index[i]][0];
			}

#ifdef MEASURE_POWER
			energy_meter_read(sample1, &em1);	 // take start reading
			record_power_time[1] = em1.current_time_stamp;
#endif

#ifdef MEASURE_POWER
			sleep(0.1);
#endif

			//*******************************test hardware time
#ifdef MEASURE_POWER
			energy_meter_read(sample1, &em1);	 // take start reading
			record_power_time[2] = em1.current_time_stamp;
#endif
			//test hardware time
			ExGBDSIPCore_FULL_hw(input_data_set, cloud_points_size, query_set, query_points_size, K, split_precise,
				query_nearest_index, query_nearest_distance, original_queryset_index, out_split_info_hw);

			for (int i = 0; i < query_points_size; i++)
			{
				result_index_hw[i] = query_nearest_index[original_queryset_index[i]][0];
				result_distance_hw[i] = query_nearest_distance[original_queryset_index[i]][0];
			}


#ifdef MEASURE_POWER
			energy_meter_read(sample1, &em1);	 // take start reading
			record_power_time[3] = em1.current_time_stamp;
#endif




			//****************************************KDTREE groundtruth
#ifdef USE_KDTREE_ON
#ifdef MEASURE_POWER
			sleep(0.1);
#endif

#ifdef MEASURE_POWER
			energy_meter_read(sample1, &em1);	 // take start reading
			record_power_time[4] = em1.current_time_stamp;
#endif

			clock_t before_kdtree = clock();
			int result_index_kdtree[k_query_set_size];
			float result_distance_kdtree[k_query_set_size];

			//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
			nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeFromMap;
			kdtreeFromMap.setInputCloud(cloud);

			clock_t build_kdtree = clock();
			for (int i = 0; i < query_points_size; i++)
			{
				//my_point_sel = query_set[i];
				pcl::PointXYZI pointSel;
				pointSel.x = query_set[i].x;
				pointSel.y = query_set[i].y;
				pointSel.z = query_set[i].z;
				std::vector<int> pointSearchInd;
				std::vector<float> pointSearchSqDis;
				pointSearchInd.resize(K);
				pointSearchSqDis.resize(K);
				kdtreeFromMap.nearestKSearch(pointSel, K, pointSearchInd, pointSearchSqDis);

				result_index_kdtree[i] = pointSearchInd[0];
				result_distance_kdtree[i] = pointSearchSqDis[0];
			}
			clock_t after_kdtree = clock();

			double time_last_kdtree = (double)(after_kdtree - before_kdtree) / CLOCKS_PER_SEC * 1000;
			double time_build_kdtree = (double)(build_kdtree - before_kdtree) / CLOCKS_PER_SEC * 1000;
			std::cout << std::endl;
			std::cout << " build kdtree with " << cloud_points_size << " dataset build kdtree time is :  " << time_build_kdtree << " ms!" << std::endl;
			std::cout << query_points_size << " queries in " << cloud_points_size << " dataset kdtree time is :  " << time_last_kdtree << " ms!" << std::endl;

#ifdef MEASURE_POWER
			energy_meter_read(sample1, &em1);	 // take start reading
			record_power_time[5] = em1.current_time_stamp;
#endif
#endif




			/////////***************************post peocess

#ifdef DEBUG_MODE

			{
				std::cout << std::endl;
				std::cout << "x_split_size: " << out_split_info_hw.x_array_size << " y_split_size: " << out_split_info_hw.y_array_size << " z_split_size: " << out_split_info_hw.z_array_size << std::endl;
				std::cout << "test out_info_hardware: " << std::endl;
				std::cout << "xmax_min: " << out_split_info_hw.xmin << " " << out_split_info_hw.xmax << std::endl;
				std::cout << "ymax_min: " << out_split_info_hw.ymin << " " << out_split_info_hw.ymax << std::endl;
				std::cout << "zmax_min: " << out_split_info_hw.zmin << " " << out_split_info_hw.zmax << std::endl;
				std::cout << "x_unit: " << out_split_info_hw.x_unit << " y_unit: " << out_split_info_hw.y_unit << " z_unit: " << out_split_info_hw.z_unit << std::endl;
				std::cout << std::endl;
			}

			//getchar();
			int error_count = 0;
			int error_count_with_kdtree = 0;
			int right_search_with_kdtree = 0;
			int error_index = 0;
			float error_threshold = (split_precise > 0) ? split_precise : 10;

			std::cout << std::endl;
			std::cout << "writing EGBDS configure to txt..." << std::endl;
			//std::string out_txt_name = "error_query_for_draw.txt";
			remove("KEGBDS_config.txt");
			std::ofstream fout2;
			fout2.open("KEGBDS_config.txt", std::ios::app);
			fout2 << out_split_info_hw.xmin << " " << out_split_info_hw.xmax << " " << out_split_info_hw.ymin << " " << out_split_info_hw.ymax
				<< " " << out_split_info_hw.zmin << " " << out_split_info_hw.zmax
				<< " " << out_split_info_hw.x_unit << " " << out_split_info_hw.y_unit << " " << out_split_info_hw.z_unit
				<< " " << k_sub_voxel_x_size << " " << k_sub_voxel_y_size << " " << k_sub_voxel_z_size
				<< std::endl;
			fout2.close();

			std::cout << "writing error_point to txt..." << std::endl;
			//std::string out_txt_name = "error_query_for_draw.txt";
			remove("error_query_for_draw.txt");
			std::ofstream fout1;
			fout1.open("error_query_for_draw.txt", std::ios::app);
			if (!fout1)
			{
				std::cout << "fail to open ordered_dataset_for_draw.txt" << std::endl;
			}
			else
			{
				std::cout << "Open error_query_for_draw.txt successfully" << std::endl;

				for (int i = 0; i < query_points_size; i++)
				{
					if ((result_distance_kdtree[i] < error_threshold * 1.1))
					{
						right_search_with_kdtree++;
					}
					if ((abs(result_distance_kdtree[i] - result_distance_hw[i]) > 0.2) && ((result_distance_kdtree[i] < error_threshold)))
					{
						std::cout << "kd_dis[" << i << "] = " << result_distance_kdtree[i] << " index: " << result_index_kdtree[i] << std::endl;
						std::cout << "hw_dis[" << i << "] = " << result_distance_hw[i] << " index: " << result_index_hw[i] << std::endl;

						error_count_with_kdtree++;
						fout1 << query_set[i].x << " " << query_set[i].y << " " << query_set[i].z << std::endl;
						fout1 << input_data_set[result_index_kdtree[i]].x << " " << input_data_set[result_index_kdtree[i]].y << " " << input_data_set[result_index_kdtree[i]].z << std::endl;
						fout1 << input_data_set[result_index_hw[i]].x << " " << input_data_set[result_index_hw[i]].y << " " << input_data_set[result_index_hw[i]].z << std::endl;
						if ((abs(result_distance_hw[i] - result_distance_sw[i]) > 0.2))
						{
							error_index = i;
							error_count = error_count + 1;
						}
					}
				}

				fout1.close();
			}

			float true_ratio = (1 - double(error_count_with_kdtree) / double(right_search_with_kdtree));


			std::cout << "error _count_with_kdtree number: " << error_count_with_kdtree << std::endl;
			std::cout << "right _count_with_kdtree number: " << right_search_with_kdtree << std::endl;
			std::cout << "successfully true ratio between kdtree and EGBDS_HW: " << true_ratio << std::endl;

			if (error_count > 0)
			{
				std::cout << std::endl;
				std::cout << std::endl;
				std::cout << "error number: " << error_count << std::endl;
				std::cout << "error , result not equal at " << error_index << "-th query" << std::endl;
				std::cout << "query is " << query_set[error_index].x << " " << query_set[error_index].y << " " << query_set[error_index].z << std::endl;
				std::cout << "gbds hw nn index: " << result_index_hw[error_index] << "  near dis: " << result_distance_hw[error_index] << std::endl;
				std::cout << "nearest point: " << input_data_set[result_index_hw[error_index]].x << " " << input_data_set[result_index_hw[error_index]].y << " " << input_data_set[result_index_hw[error_index]].z << std::endl;
				std::cout << "gbds sw nn index: " << result_index_sw[error_index] << "  near dis: " << result_distance_sw[error_index] << std::endl;
				std::cout << "nearest point: " << input_data_set[result_index_sw[error_index]].x << " " << input_data_set[result_index_sw[error_index]].y << " " << input_data_set[result_index_sw[error_index]].z << std::endl;
			}
			else
				std::cout << "ccorrect hw result!" << std::endl;

#endif

			//////////////////****************OUTPUT DATA TO FILE
			std::cout << std::endl;
			std::cout << std::endl;
			std::cout << "writing data to txt..." << std::endl;
			std::string out_txt_name;
			if(test_same_dataset_flag)
				out_txt_name = "TEST_EGBDS_SAME_DATASET.txt";
			else
				out_txt_name = "TEST_EGBDS_SAME_QUERYSET.txt";
			std::string test_type = "TEST_EGBDS_PL";	//EGBDS_WITH_PL   BASELINE_PURE_PS   BASELINE_WITH_PL
			std::ofstream fout;
			fout.open(out_txt_name.c_str(), std::ios::app);
			if (!fout)
			{
				std::cout << "fail to open KNNTEST.txt" << std::endl;
				return -1;
			}
			else
			{
				std::cout << "Open TEST_EGBDS_PL.txt successfully" << std::endl;
				fout << cloud_points_size << " " << query_points_size << " " << K << " ";
				fout << out_split_info_sw.build_time << " " << out_split_info_sw.search_time << " ";
				fout << out_split_info_hw.build_time << " " << out_split_info_hw.search_time << " "<< true_ratio << " "; //
#ifdef MEASURE_POWER
				fout << record_power_time[0] << " " << record_power_time[1] << " " << record_power_time[2] << " " <<
					record_power_time[3] << " " << record_power_time[4] << " " << record_power_time[5] << " " << std::endl;
#else
				fout << std::endl;
#endif
				fout.close();
			}
		}//loop for j

#ifdef USE_SDX
		sds_free(input_data_set);
		sds_free(query_nearest_distance);
		sds_free(query_nearest_index);
		sds_free(my_point_sel);
		sds_free(query_set);
		sds_free(original_queryset_index);
#endif
		
	}//loop for files name

#ifdef MEASURE_POWER
	energy_meter_stop(sample1);  	// stops sampling
	energy_meter_printf(sample1, stderr);  // print total results
	energy_meter_destroy(sample1);     // clean up everything
#endif


#ifdef USE_SDX
#else
	getchar();
#endif
	return 0;
}

#endif
