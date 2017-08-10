KUDAN=..
#The variable 'KUDAN' should define the path to the KudanSLAM library

#OpenCV is a dependency of these demos, its location needs to be specified (not included in KudanSLAM!). This is usually the folder containing the include and lib folders
OPENCV= /usr/local/Cellar/opencv3/3.2.0
# This is the path to the libs in OpenCV, it's probably the following but can be changed
OPENCV_LIB_PATH=$(OPENCV)/lib


CC=g++
CFLAGS=-std=c++11 -O3 -Wfatal-errors


OPENCV_LIBS = -lopencv_calib3d \
				-lopencv_core \
				-lopencv_features2d \
				-lopencv_flann \
				-lopencv_hal \
				-lopencv_highgui \
				-lopencv_imgcodecs \
				-lopencv_imgproc \
				-lopencv_ml \
				-lopencv_objdetect \
				-lopencv_video \
				-lopencv_videoio \
				
# This builds a minimal example of how which interfaces with the library (no dependencies)
TinyDemo: TinyDemo.cpp
	$(CC) -o TinyDemo TinyDemo.cpp -I $(KUDAN)/include -L $(KUDAN)/lib -lKudanSLAM -lpthread -L $(OPENCV_LIB_PATH) $(OPENCV_LIBS) $(CFLAGS)
	
# This builds a simple demo which loads a video and runs/shows SLAM (in 2D) via the library
VideoDemo: VideoDemo.cpp
	$(CC) -o VideoDemo VideoDemo.cpp -I $(KUDAN)/include -L $(KUDAN)/lib -lKudanSLAM -lpthread  -I $(OPENCV)/include -L $(OPENCV_LIB_PATH)  $(OPENCV_LIBS)  $(CFLAGS)
	
all: TinyDemo VideoDemo

.DEFAULT_GOAL := all	

	
.PHONY : clean
clean :
	rm -r ./TinyDemo ./VideoDemo
	
	
.DEFAULT_GOAL := all	
