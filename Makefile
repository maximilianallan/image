examples/mono:examples/mono.cpp image/image.hpp
	g++ `pkg-config --libs --cflags opencv` examples/mono.cpp -o examples/mono
clean:
	\rm examples/mono