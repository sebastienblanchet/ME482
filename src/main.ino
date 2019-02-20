/*
 * 
 * ME482: Main Arduino Code
 *
 * Author: Sebastien Blanchet
 *
 * Date: 02/20/2019
 * 
 */

/* PINOUT */
#define HWINT        2   // hardware interupt
#define KHEATERS     4   // heaters relay pin
#define BUZZ         5   // buzzer PWM line
#define LEDPIN       6   // Description
#define KFAN         7   // Description
#define STARTSW      8   // Description
#define STEPENABLE   9   // Description
#define STEPDIR      10  // Description
#define STEPPULSE    11  // Description
#define THERM0       A0  // thermistor 1 reading
#define THERM1       A1  // thermistor 1 reading



/* MAIN ROUTINE */
void setup()
{
    // Define digital pins
    pinMode(HWINT,      INPUT );
    pinMode(KHEATERS,   OUTPUT);
    pinMode(BUZZ,       OUTPUT);
    pinMode(LEDPIN,     OUTPUT);
    pinMode(KFAN,       OUTPUT);
    pinMode(STARTSW,    INPUT );
    pinMode(STEPENABLE, OUTPUT);
    pinMode(STEPDIR,    OUTPUT);
    pinMode(STEPPULSE,  OUTPUT);
}

void loop()
{
    // Main
}