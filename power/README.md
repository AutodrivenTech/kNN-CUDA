# Add power monitor for GPU using NVML Library

> [NVIDIA Management Library](https://developer.nvidia.com/nvidia-management-library-nvml) 
A C-based API for monitoring and managing various states of the NVIDIA GPU devices. It provides a direct access to the queries and commands exposed via nvidia-smi. The runtime version of NVML ships with the NVIDIA display driver, and the SDK provides the appropriate header, stub libraries and sample applications. Each new version of NVML is backwards compatible and is intended to be a platform for building 3rd party applications.

For computing the power usage of the GPU, we use the NVML to sampling the power usage every small time gap, the integral those data to get a estimated power consuming.

