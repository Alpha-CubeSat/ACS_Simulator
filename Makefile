CXX = g++
CXXFLAGS = -Ilib/StarshotACS_Ert_rtw
SOURCES = src/main.cpp lib/StarshotACS_Ert_rtw/StarshotACS.cpp lib/StarshotACS_Ert_rtw/StarshotACS_data.cpp lib/StarshotACS_Ert_rtw/ert_main.cpp lib/Plantv50_ert_rtw/ert_main.cpp lib/Plantv50_ert_rtw/Plantv50_data.cpp lib/Plantv50_ert_rtw/Plantv50.cpp
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
