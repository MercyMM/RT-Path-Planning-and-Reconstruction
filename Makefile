LIBSRCS = DJI_API.cpp\
          DJI_App.cpp\
          DJI_Camera.cpp \
          DJI_Codec.cpp \
          DJI_Flight.cpp \
          DJI_Follow.cpp \
          DJI_HardDriver.cpp \
          DJI_HotPoint.cpp \
          DJI_Link.cpp \
          DJI_Memory.cpp \
          DJI_Mission.cpp \
          DJI_VirtualRC.cpp \
          DJI_WayPoint.cpp 


SRCS = main.cpp \
       DJI_utility.cpp\
       LinuxThread.cpp \
       LinuxSerialDevice.cpp\
       LinuxSetup.cpp\
       LinuxFlight.cpp\
       LinuxCleanup.cpp\
       LinuxWaypoint.cpp\
       ReadUserConfig.cpp\
	descriptor.cpp \
	elas.cpp\
	main.cpp\
	matrix.cpp\
	triangle.cpp\
	vibe.cpp\
       $(LIBSRCS)

OBJS = $(SRCS:.cpp=.o)

#TARGET = main
TARGET = ./bin/main

#LIBDIR = ../../lib
LIBDIR = osdk-core

#VPATH = $(LIBDIR)/inc:$(LIBDIR)/src:$(DJISCRIPTDIR)/inc:$(DJISCRIPTDIR)/src

LIBLITMUS = /usr/src/liblitmus/liblitmus

CC = g++
INC = -I$(LIBDIR)/inc  -Iinc
CXXFLAGS = --std=c++11 $(INC) -DHAVE_OPENCV -pthread  -I${LIBLITMUS}/include -I${LIBLITMUS}/arch/arm/include -I${LIBLITMUS}/arch/arm/include/uapi -I${LIBLITMUS}/arch/arm/include/generated/uapi -mfpu=neon -fpermissive -w -O3
#CXXFLAGS = --std=c++11 $(INC) -DHAVE_OPENCV -pthread  -I${LIBLITMUS}/include -I${LIBLITMUS}/arch/arm/include -I${LIBLITMUS}/arch/arm/include/uapi -I${LIBLITMUS}/arch/arm/include/generated/uapi -mfpu=neon -fpermissive -w -O3
#LDFLAGS = -Wl,-rpath,./ -lpthread -lrt -L./ -lDJI_guidance -lkernel -L/usr/src/liblitmus/liblitmus/ -llitmus  -L/usr/local/lib/ -lusb-1.0 `pkg-config --cflags --libs opencv`
LDFLAGS = -Wl,-rpath,/home/ubuntu/sd/TK1-bak/pathPlan-test -lpthread -lrt -L./ -lDJI_guidance -lkernel  -L/usr/local/lib/ -lusb-1.0 -L/usr/local/cuda-6.5/targets/armv7-linux-gnueabihf/lib/ `pkg-config --cflags --libs opencv`

$(TARGET) : $(addprefix objs/, $(OBJS))
	$(CC) -o $@ $(CXXFLAGS) $^  $(LDFLAGS)

objs/main.o : src/main.cpp 
	$(CC) -o $@ -c $< $(CXXFLAGS) 

objs/%.o : src/%.cpp inc/%.h
	$(CC) -o $@ -c $< $(CXXFLAGS)

objs/%.o : $(LIBDIR)/src/%.cpp
	$(CC)  -o $@ -c $< $(CXXFLAGS) 

#$(LIBDIR)/inc/%.h $(INC)

kernel:
        /usr/local/cuda-6.5/bin/nvcc -o objs/kernel.o -c src/kernel.cu -w --gpu-architecture=compute_32 --gpu-code=compute_32 -DOPENCV -I/usr/local/cuda-6.5/include/ -Iinc --compiler-options "-Wall -Wfatal-errors -Ofast -fPIC"; \
	/usr/local/cuda-6.5/bin/nvcc -shared -o libkernel.so -L/usr/local/cuda-6.5/lib -lcuda -lcudart -lcublas -lcurand objs/kernel.o



clean :
	rm -f objs/* bin/*
