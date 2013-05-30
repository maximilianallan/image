examples/mono:examples/mono.cpp image/image.hpp
	g++ -I/cygdrive/c/"Program Files"/opencv/build/install/include/opencv/ -I/cygdrive/c/"Program Files"/opencv/build/install/include/ -L/cygdrive/c/"Program Files"/opencv/build/install/lib -lopencv_core249 examples/mono.cpp -o examples/mono
clean:
	\rm examples/mono