#include "Arduino.h"
#include <SD.h>
#include <SPI.h>


//Initializes the SD card
void DataLogSetup();

//Opens the Data.txt file, writes data, close the file when exits
void DataLog(double Data[], int size);
