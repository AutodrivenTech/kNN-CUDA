NVML_LIB := /usr/local/cuda/lib64
NVML_INCLUDE := /usr/local/cuda/include
LDFLAGS := -lnvidia-ml -L $(NVML_LIB) -pthread
CFLAGS := -I $(NVML_INCLUDE)

power: power.o
	$(CC)  $< -o $@ $(LDFLAGS)

power.o:
	$(CC) -c $(CFLAGS) -o $@ power.c

clean:
	-@rm -f power.o
	-@rm -f power