#include "DataLogging.hpp"

const int chipSelect = BUILTIN_SDCARD;
File DataFile;


void DataLogSetup(){
    
    if (!SD.begin(chipSelect)) {
        Serial.println("SD Card initialization failed!");
        return;
    }

    Serial.println("SD Card initialization done.");
    //SD.remove("Data.txt");
    DataFile = SD.open("Data.txt", FILE_WRITE);
    
    if(DataFile){
        DataFile.println("---RESTART---");
    }
    
    DataFile.close();
}

void DataLog(double Data[], int size)
{
    DataFile = SD.open("Data.txt", FILE_WRITE);
    if (DataFile) {
        for(int i = 0 ; i< size;i++) {
            DataFile.print(Data[i]);
            DataFile.print(", ");

            Serial.print(Data[i]);
            Serial.print(", ");
        }
        DataFile.println();
        Serial.println();

        DataFile.close();
        //Serial.println("done.");
    } else {
        // if the file didn't open, print an error:
        Serial.println("error opening");
    }
    DataFile.close();
}