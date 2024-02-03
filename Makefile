CXX = g++
CXXFLAGS = -Ilib/StarshotACS_Ert_rtw  -Ilib/Plant_Ert_rtw 

SOURCES = src/main.cpp lib/ACS_libs/StarshotACS_Ert_rtw/StarshotACS.cpp lib/ACS_libs/StarshotACS_Ert_rtw/StarshotACS_data.cpp lib/ACS_libs/Plant_ert_rtw/Plant_data.cpp lib/ACS_libs/Plant_ert_rtw/Plant.cpp
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
