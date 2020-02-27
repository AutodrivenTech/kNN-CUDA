NVML_LIB := /usr/local/cuda/lib64
NVML_INCLUDE := /usr/local/cuda/include
LDFLAGS := -lnvidia-ml -L $(NVML_LIB)
CFLAGS := -I $(NVML_INCLUDE)

default:
	make power.o
	make power

power.o:
	$(CC) -c $(CFLAGS) -o $@ power.c

power: power.o
	$(CC)  $< -o $@ $(LDFLAGS)

clean:
	-@rm -f power.o
	-@rm -f power