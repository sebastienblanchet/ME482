/*
 *
 * ME482: Main Arduino Code
 *
 *
 * MIT License
 *
 * Copyright (c) 2019 Sebastien Blanchet
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */


/*************************************************************************************************
**************************************************************************************************/

/* Hardware specifics */
#define MAXCOUNT 1023    // Max ADC counts
#define VREF     5       // Arduino reference voltage
#define RNOM     110000  // Thermistor resistance at 25 deg C
#define BCOEF    3950    // Thermistor B value for Steinhart equation
#define RSERIES  10000   // Pull up series resistor for thermistor sense
#define ISENSK   10      // Current sensor gain (i.e 1 V / 100mV/A)
#define ISENSOFF 25      // Current sensor offset (i.e 2.5 V / 100 mV/A)
#define TNOM     25      // Nominal temperature room temp 25 def C
#define STEPS    400     // Number of steps per rev

/* Pinout for hardware */
#define HWINT        2   // hardware interrupt
#define KHEATERS     4   // heaters relay pin
#define BUZZ         5   // buzzer PWM line
#define LEDPIN       6   // controls LED in UI
#define KFAN         7   // controls relay switch for fan
#define STARTSW      8   // takes user input 
#define STEPENABLE   9   // enable the stepper motor
#define STEPDIR      10  // enable CW or CCW motor dir
#define STEPPULSE    11  // PWM line for motor
#define LIMITSW      12  // Limit switch for motor return position 
#define THERM0       A0  // thermistor 1 reading
#define THERM1       A1  // thermistor 2 reading
#define ISENS        A2  // motor current reading


/*************************************************************************************************

GLOBALS

**************************************************************************************************/

/* Global variables */
boolean pulse = LOW;
const int FWD = 578;
const int BWD = 448;
const int speedMAX = 200;

/*************************************************************************************************

HELPER FUNCTIONS

**************************************************************************************************/

/* Get actual current sensor value */
// TODO: validate getIsensA
float getIsensA(uint32_t maxcount)
{
    static uint32_t counter = 0;
    static float current = 0;

    if ( counter == 0 )
    {
        // Map analog counts to voltage
        float voltIsens = map(analogRead(ISENS), 0, MAXCOUNT, 0, VREF);
        // Calculate measured current
        current = (ISENSK * voltIsens) - ISENSOFF;
        // Reset counter to MAX
        counter = maxcount;
    }
    else
    {
        // Decrement counter
        counter--;
    }

    return current;
}


uint32_t getMotorTus(uint32_t motorSpeedRPM)
{
    uint32_t motorTus;

    motorTus = (30000000) / (STEPS * motorSpeedRPM);

    return motorTus;
}

uint32_t intervalms = getMotorTus(speedMAX);

void motorPulse()
{   
    pulse = !pulse;
    digitalWrite(STEPPULSE, pulse);
    delayMicroseconds(intervalms);
}


/*************************************************************************************************

MAIN

**************************************************************************************************/

/* Setup the Arduino hardware */
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
    digitalWrite(STEPENABLE, LOW);
}


/* Main Routine */
void loop()
{
    int speedCounts = analogRead(A5);

    if ( speedCounts > FWD )
    {
        digitalWrite(STEPDIR, HIGH);
        motorPulse();

    }
    else if ( speedCounts <  BWD )
    {              
        digitalWrite(STEPDIR, LOW);
        motorPulse();

    }

}
