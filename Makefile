EXE=com.out
objects= build/main.o

LIBS = -lpthread -I/usr/include/eigen3 -lorocos-kdl
CXX = g++
CXXFLAGS = --std=c++11

default: $(EXE)


build/%.o: ./%.cpp
	$(CXX) -c  $(CXXFLAGS) $^ -o $@ $(LIBS)

$(EXE): $(objects)
	$(CXX) $(CXXFLAGS) $(objects) -o $(EXE) $(LIBS)


clean:
	rm $(EXE) $(objects)
