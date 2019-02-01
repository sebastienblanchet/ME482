/*
 * Example using NEMA23 and TB6600 driver
 *
 * modified from :
 * https://www.instructables.com/id/Nema-Stepper-Motor-23-With-Tb6600-Driver-With-Ardu/
 * 
 */

/* Pinout */
#define STEPENABLE 2
#define STEPDIR    3
#define STEPPULSE  4

/* Globals */
const int intervalms = 350;
boolean pulse = LOW;


/*
 * Motor pulse is a simply a 50% duty cycle square wave i.e.
 * a clock which then gets handled by driver
 */
void motorPulse()
{   
    pulse = !pulse;
    digitalWrite(STEPPULSE, pulse);
    delayMicroseconds(intervalms);
}

void setup()
{
    pinMode(STEPENABLE, OUTPUT);
    pinMode(STEPDIR, OUTPUT);
    pinMode(STEPPULSE, OUTPUT);
    digitalWrite( STEPENABLE, LOW);
    digitalWrite(STEPDIR, HIGH);
    digitalWrite(STEPPULSE, HIGH);
}

void loop()
{
    motorPulse();
}