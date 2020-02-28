#include "power.h"
#define MAX_NUM_OF_DATA 10000000
#define TIME_STEP  10000
#define GPU_INDEX 0

static nvmlDevice_t device;
unsigned int data[MAX_NUM_OF_DATA];
static unsigned int time_step = TIME_STEP;
static unsigned int gpu_index = GPU_INDEX;

static pthread_t power_poll_thread;
static bool poll_thread_status = false;


void nvml_api_init()
{
    nvmlReturn_t result;
    unsigned int device_count;

    // int the library
    result = nvmlInit();
    if (NVML_SUCCESS != result)
    { 
        fprintf(stderr, "Failed to initialize NVML: %s\n", nvmlErrorString(result));
        exit(1);
    }

    result = nvmlDeviceGetCount(&device_count);
    if(NVML_SUCCESS != result){
        fprintf(stderr, "Failed to query device count: %s\n", nvmlErrorString(result));
        exit(1);
    }
    if(device_count - 1 < gpu_index){
        fprintf(stderr, "GPU index %d out of device_count %d", gpu_index, device_count);
        exit(1);
    }

    nvmlDevice_t device;
    char name[NVML_DEVICE_NAME_BUFFER_SIZE];
    nvmlPciInfo_t pci;
        result = nvmlDeviceGetHandleByIndex(gpu_index, &device);
        if (NVML_SUCCESS != result)
        { 
            fprintf(stderr, "Failed to get handle for device %i: %s\n", gpu_index, nvmlErrorString(result));
            result = nvmlShutdown();
            if (NVML_SUCCESS != result)
            fprintf(stderr, "Failed to shutdown NVML: %s\n", nvmlErrorString(result));
            exit(1);
        }

        result = nvmlDeviceGetName(device, name, NVML_DEVICE_NAME_BUFFER_SIZE);
        if (NVML_SUCCESS != result)
        { 
            fprintf(stderr, "Failed to get name of device %i: %s\n", gpu_index, nvmlErrorString(result));
            result = nvmlShutdown();
            if (NVML_SUCCESS != result)
                fprintf(stderr, "Failed to shutdown NVML: %s\n", nvmlErrorString(result));
            exit(1);
        }
        
        // pci.busId is very useful to know which device physically you're talking to
        // Using PCI identifier you can also match nvmlDevice handle to CUDA device.
        result = nvmlDeviceGetPciInfo(device, &pci);
        if (NVML_SUCCESS != result)
        { 
            fprintf(stderr, "Failed to get pci info for device %i: %s\n", gpu_index, nvmlErrorString(result));
            result = nvmlShutdown();
            if (NVML_SUCCESS != result)
                fprintf(stderr, "Failed to shutdown NVML: %s\n", nvmlErrorString(result));
            exit(1);
        }
        fprintf(stdout, "nvml_api_init() success!");
        fprintf(stdout, "%d. %s [%s]\n", gpu_index, name, pci.busId);
}

void nvml_api_close(){
    nvmlReturn_t result;
    result = nvmlShutdown();
    if (NVML_SUCCESS != result)
        fprintf(stderr, "Failed to shutdown NVML: %s\n", nvmlErrorString(result));
    fprintf(stdout, "nvml_api_close() success.\n");
}

void *nvml_power_monitor(void* ptr){
    unsigned int power_level = 0;
    unsigned int count = 0;
    nvmlReturn_t result;
    nvmlEnableState_t pmmode;

    while (poll_thread_status){
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, 0);
        result = nvmlDeviceGetHandleByIndex(gpu_index, &device);
        result = nvmlDeviceGetPowerManagementMode(device, &pmmode);
        if(NVML_SUCCESS != result){
            fprintf(stderr, "error power mode : %s\n", nvmlErrorString(result));
        }
        if(pmmode == NVML_FEATURE_ENABLED){
            result = nvmlDeviceGetPowerUsage(device, &power_level);
            if(NVML_SUCCESS != result){
                fprintf(stderr, "error get power : %s\n", nvmlErrorString(result));
            }
        }
        data[count] = power_level;
        // fprintf(stdout, "power %d\n", power_level);
        count++;
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, 0);
        usleep(time_step);
    }
    pthread_exit(0);
}

void nvml_monitor_start(){
    poll_thread_status = true;
    const char* message = "NVML monitor message";
    int iret = pthread_create(&power_poll_thread, NULL, nvml_power_monitor, (void*) message);
    if(iret){
        fprintf(stderr, "Error to create pthread with retrun code:%d\n", iret);
        exit(0);
    }
}

void nvml_monitor_stop(){
    poll_thread_status = false;
    pthread_join(power_poll_thread, NULL);
}

double integral_power_consuming(){
    unsigned int count;
    double result;
    for(count = 0; count < MAX_NUM_OF_DATA; count++){
        if(!data[count]){
            break;
        }
        result += 1.0 / (double)time_step * (double)data[count] / 1000.0;
    }
    return result / (double)count;
}

int main()
{
    nvml_api_init(0);
    nvml_monitor_start();
    sleep(10);
    nvml_monitor_stop();
    nvml_api_close();

    unsigned int count;
    for(count =0; count <= MAX_NUM_OF_DATA; count++){
        if(!data[count]){
            break;
        }
    }
    fprintf(stdout, "The data count is %d", count);
    double power = integral_power_consuming();
    fprintf(stdout, "the power consumping is %0.5f", power);
    return 0;
}