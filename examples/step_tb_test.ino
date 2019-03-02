/*
 * Example using NEMA23 and TB6600 driver
 *
 * modified from :
 * https://www.instructables.com/id/Nema-Stepper-Motor-23-With-Tb6600-Driver-With-Ardu/
 * 
 */

/* Pinout */
#define STEPENABLE 9
#define STEPDIR    10
#define STEPPULSE  11
#define STEPS 400

/* Globals */
boolean pulse = LOW;


uint32_t getMotorTus(uint32_t motorSpeedRPM)
{
    uint32_t motorTus;

    motorTus = (60000000) / (STEPS * motorSpeedRPM);

    return motorTus;
}

uint32_t intervalms = getMotorTus(100);

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
    digitalWrite(STEPENABLE, LOW);
    digitalWrite(STEPDIR, HIGH);
    digitalWrite(STEPPULSE, HIGH);
}

void loop()
{
    motorPulse();
}