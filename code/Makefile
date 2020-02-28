LDFLAGS := -L/usr/local/cuda/lib64 -lcuda -lcublas -lcudart -lnvidia-ml -pthread
OTHER_FLAGS = -Wno-deprecated-gpu-targets
CFLAGS := -I../power/  -I/usr/local/cuda/include
NVCC := /usr/local/cuda/bin/nvcc

.PHONY: all test

all: test

power.o:
	$(CXX) -c $(CFLAGS) -o $@ ../power/power.c

knncuda.o:
	$(NVCC) -c $(CFLAGS) -o $@ knncuda.cu 

# test:
# 	$(NVCC) -o test $(CFLAGS) $(OTHER_FLAGS) test.cpp ../power/power.c knncuda.cu  $(LDFLAGS)

test.o:
	$(CXX) -c $(CFLAGS) -o $@ test.cpp

test: test.o knncuda.o power.o
	$(CXX) -o test test.o knncuda.o power.o $(LDFLAGS) $(OTHER_FLAGS)

clean:
	rm test test.o knncuda.o power.o
