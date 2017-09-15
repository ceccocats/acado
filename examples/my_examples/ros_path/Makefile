all: ros_path

ROS_LIBS=-lroscpp -lrosconsole -lroscpp_serialization -lrostime -ltf
ROS_LIBS_DIR=/opt/ros/indigo/lib

LIBS=-lboost_system -lm -lstdc++ $(ROS_LIBS)
ACADO_LIB=/home/ron/Documents/mpc/ACADOtoolkit/examples/my_examples/poly_path_code/libacado_exported_rti.a

LFLAGS=-L$(ROS_LIBS_DIR) -L. $(LIBS) $(ACADO_LIB)

ros_path: ros_path.cpp
	g++ ros_path.cpp $(CFLAGS) $(LFLAGS) -o ros_path -std=gnu++0x

clean:
	rm ros_path *~


