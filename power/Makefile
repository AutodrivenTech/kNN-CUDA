NVCC := /usr/local/cuda/bin/nvcc
NVML_LIB := /usr/local/cuda/lib64
NVML_INCLUDE := /usr/local/cuda/include
LDFLAGS := -L$(NVML_LIB) -lnvidia-ml -lcuda -lcudart  -pthread
CFLAGS := -I $(NVML_INCLUDE) -I /usr/local/cuda/samples/common/inc -I.
OTHER_FLAGS := 

# test.out: power.o test.o
# 	$(CC) -o $@ $(OTHER_FLAGS) $(CFLAGS) $< $(LDFLAGS)

# test.o:
# 	$(NVCC) -c $(CFLAGS) -o $@ test.cu

power.o:
	$(NVCC) -c $(CFLAGS) -o $@ power.c

clean:
	-@rm -f power.o
	-@rm -f test.o
	-@rm -rf test.out