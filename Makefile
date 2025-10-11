CXXFLAGS=-Wall -O3 -g -fno-strict-aliasing
BINARIES=droplet

RGB_INCDIR=include
RGB_LIBDIR=lib
LDFLAGS+=-L$(RGB_LIBDIR) -lrgbmatrix -lrt -lm -lpthread

all : $(BINARIES)


droplet : droplet.o $(RGB_LIBRARY)
	$(CXX) $(CXXFLAGS) droplet.o -o $@ $(LDFLAGS)


%.o : %.cpp
	$(CXX) -I$(RGB_INCDIR) $(CXXFLAGS) -DADAFRUIT_RGBMATRIX_HAT -c -o $@ $<

clean:
	rm -f *.o $(OBJECTS) $(BINARIES)

