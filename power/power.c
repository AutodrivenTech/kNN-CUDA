#include "power.h"

const char * convertToComputeModeString(nvmlComputeMode_t mode)
{
    switch (mode)
    {
        case NVML_COMPUTEMODE_DEFAULT:
            return "Default";
        case NVML_COMPUTEMODE_EXCLUSIVE_THREAD:
            return "Exclusive_Thread";
        case NVML_COMPUTEMODE_PROHIBITED:
            return "Prohibited";
        case NVML_COMPUTEMODE_EXCLUSIVE_PROCESS:
            return "Exclusive Process";
        default:
            return "Unknown";
    }
}

int main()
{
    nvmlReturn_t result;
    unsigned int device_count, i;

    // First initialize NVML library
    result = nvmlInit();
    if (NVML_SUCCESS != result)
    { 
        printf("Failed to initialize NVML: %s\n", nvmlErrorString(result));

        printf("Press ENTER to continue...\n");
        getchar();
        return 1;
    }

    result = nvmlDeviceGetCount(&device_count);
    if (NVML_SUCCESS != result)
    { 
        printf("Failed to query device count: %s\n", nvmlErrorString(result));
        goto Error;
    }
    printf("Found %d device%s\n\n", device_count, device_count != 1 ? "s" : "");

    printf("Listing devices:\n");    
    for (i = 0; i < device_count; i++)
    {
        nvmlDevice_t device;
        char name[NVML_DEVICE_NAME_BUFFER_SIZE];
        nvmlPciInfo_t pci;
        nvmlComputeMode_t compute_mode;

        // Query for device handle to perform operations on a device
        // You can also query device handle by other features like:
        // nvmlDeviceGetHandleBySerial
        // nvmlDeviceGetHandleByPciBusId
        result = nvmlDeviceGetHandleByIndex(i, &device);
        if (NVML_SUCCESS != result)
        { 
            printf("Failed to get handle for device %i: %s\n", i, nvmlErrorString(result));
            goto Error;
        }

        result = nvmlDeviceGetName(device, name, NVML_DEVICE_NAME_BUFFER_SIZE);
        if (NVML_SUCCESS != result)
        { 
            printf("Failed to get name of device %i: %s\n", i, nvmlErrorString(result));
            goto Error;
        }
        
        // pci.busId is very useful to know which device physically you're talking to
        // Using PCI identifier you can also match nvmlDevice handle to CUDA device.
        result = nvmlDeviceGetPciInfo(device, &pci);
        if (NVML_SUCCESS != result)
        { 
            printf("Failed to get pci info for device %i: %s\n", i, nvmlErrorString(result));
            goto Error;
        }

        printf("%d. %s [%s]\n", i, name, pci.busId);

        // Get the compute mode of the device which indicates CUDA capabilities.
		result = nvmlDeviceGetComputeMode(device, &compute_mode);
		if (NVML_ERROR_NOT_SUPPORTED == result)
		{
			printf("This is not a CUDA-capable device.\n");
		}
		else if (NVML_SUCCESS != result)
		{
			printf("Failed to get compute mode for device %i: %s\n", i, nvmlErrorString(result));
			exit(0);
		}
    }

    nvmlEnableState_t pmmode;
    nvmlDevice_t device;
    unsigned int power_level;

    result = nvmlDeviceGetHandleByIndex(0, &device);

    result = nvmlDeviceGetPowerManagementMode(device, &pmmode);
    if(NVML_SUCCESS != result){
        printf("error : %s\n", nvmlErrorString(result));
        exit(0);
    }
    if(pmmode == NVML_FEATURE_ENABLED){
		while(1){
			result = nvmlDeviceGetPowerUsage(device, &power_level);
			printf("%.4lf\n", (power_level)/1000.0);
			usleep(10000);	
		}
    }
        
    result = nvmlShutdown();
    if (NVML_SUCCESS != result)
        printf("Failed to shutdown NVML: %s\n", nvmlErrorString(result));

    printf("All done.\n");

    printf("Press ENTER to continue...\n");
    getchar();
    return 0;

Error:
    result = nvmlShutdown();
    if (NVML_SUCCESS != result)
        printf("Failed to shutdown NVML: %s\n", nvmlErrorString(result));

    printf("Press ENTER to continue...\n");
    getchar();
    return 1;
}
