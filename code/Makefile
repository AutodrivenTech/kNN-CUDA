LDFLAGS := -L/usr/lib/x86_64-linux-gnu/ -L/usr/local/cuda/lib64 -lboost_system -lgflags -lcuda -lcublas -lcudart -lnvidia-ml -pthread -lpcl_io -lOpenNI -lOpenNI2 -lpcl_octree -lpcl_common
OTHER_FLAGS = -Wno-deprecated-gpu-targets
CFLAGS := -I../power/  -I/usr/local/cuda/include -I/usr/include/pcl-1.8 -I/usr/include/ni -I/usr/include/openni2 -I/usr/include/eigen3
NVCC := /usr/local/cuda/bin/nvcc

.PHONY: all test

all: test

power.o:
	$(CXX) -g -c $(CFLAGS) -o $@ ../power/power.c

knncuda.o:
	$(NVCC) -c $(CFLAGS) -o $@ knncuda.cu 

# test:
# 	$(NVCC) -o test $(CFLAGS) $(OTHER_FLAGS) test.cpp ../power/power.c knncuda.cu  $(LDFLAGS)

test.o:
	$(CXX) -g --std=c++17 -c $(CFLAGS) -o $@ main.cc

test: test.o knncuda.o power.o
	$(CXX) -o test test.o knncuda.o power.o $(LDFLAGS) $(OTHER_FLAGS)

clean:
	rm test test.o knncuda.o power.o
