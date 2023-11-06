CXX = g++
CXXFLAGS =-Ilib -std=c++14

SOURCES = src/main.cpp lib/StarshotACS_Ert_rtw/StarshotACS.cpp lib/StarshotACS_Ert_rtw/StarshotACS_data.cpp lib/Plant_ert_rtw/Plant_data.cpp lib/Plant_ert_rtw/Plant.cpp lib/ekf/ekf.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = main

.PHONY: all clean

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)
	rm output/test.txt
