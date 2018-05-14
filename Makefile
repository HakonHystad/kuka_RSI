EXE=com.out
objects= build/main.o

LIBS = 
CXX = g++
CXXFLAGS = --std=c++11

default: $(EXE)


build/%.o: ./%.cpp
	$(CXX) -c  $(CXXFLAGS) $^ -o $@

$(EXE): $(objects)
	$(CXX) $(CXXFLAGS) $(objects) -o $(EXE) $(LIBS)


clean:
	rm $(EXE) $(objects)
