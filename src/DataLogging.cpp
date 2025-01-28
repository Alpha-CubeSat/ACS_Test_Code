#include "DataLogging.hpp"

const int chipSelect = BUILTIN_SDCARD;
File DataFile;


void DataLogSetup(String s){
    
    if (!SD.begin(chipSelect)) {
        Serial.println("SD Card initialization failed!");
        return;
    }

    Serial.println("SD Card initialization done.");

    DataFile = SD.open(s.append(".txt").c_str(), FILE_WRITE);

    DataFile.println("---START---");
    
    DataFile.close();
}

void DataLog(double Data[], int size, String s)
{
    DataFile = SD.open(s.append(".txt").c_str(), FILE_WRITE);
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