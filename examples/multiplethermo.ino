#include "Adafruit_MAX31855.h"

int thermoDO = 3;
int thermoCS = 4;
int thermoCLK = 5;

#define NUMBER_OF_THERMOCOUPLES 2
int sharedThermoPins[] = {thermoDO, thermoCS};
int thermoCLKs[NUMBER_OF_THERMOCOUPLES] = {thermoCLK, thermoCLK + 1};
Adafruit_MAX31855  *thermocouples[NUMBER_OF_THERMOCOUPLES];

void setup()
{
    Serial.begin(9600);

    Serial.println("MAX31855 array test");
    // wait for MAX chip to stabilize
    delay(500);

    for (int i = 0; i < NUMBER_OF_THERMOCOUPLES; i++)
    {
        thermocouples[i] = new Adafruit_MAX31855(thermoCLKs[i], sharedThermoPins[1], sharedThermoPins[0]);
    }
}

void loop() {

    // basic readout test, just print the current temp
    for (int i = 0; i < NUMBER_OF_THERMOCOUPLES; i++)
    {
        Serial.print("Internal Temp "); Serial.print(i); Serial.print(" = ");
        Serial.println(thermocouples[i]->readInternal());

        double c = thermocouples[i]->readCelsius();
        if (isnan(c))
        {
            Serial.print("Something wrong with thermocouple"); Serial.print(i); Serial.println("!");
        } 
        else 
        {
            Serial.print("C "); Serial.print(i); Serial.print(" = ");
            Serial.println(c);
        }

        //Serial.print("F ");Serial.print(i); Serial.print(" = ");
        //Serial.println(thermocouples[i].readFarenheit());
    }

    delay(1000);
}