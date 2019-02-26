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


/* Pinout for hardware */
#define HWINT        2   // hardware interupt
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


/* Global variables */
unsigned tDelta;


/* Helper function for pin pulse */
void pinPulse(int pinNum, int delayMs)
{
    digitalWrite(pinNum, HIGH);
    delay(delayMs);
    digitalWrite(pinNum, LOW);
    delayMs(delayMs);
}


/* Helper function to read avg temp count */
int avgTemp()
{
    int avgTemp;

    avgTemp += analogRead(THERM0);
    avgTemp += analogRead(THERM1);
    avgTemp /= 2;

    return avgTemp;
}


/* Wind the motor to stall */
void windMotor()
{
    const int intervalms = 250;         // PWM frequency of (i.e. == RPM)
    const int maxCurrentCounts = 200;   // Corresponds to 2.5 A from motor

    digitalWrite(STEPENABLE, HIGH); // Write enable
    digitalWrite(STEPDIR, HIGH);    // Set CW motor direction
    digitalWrite(STEPPULSE, LOW);   // Set pulse low

    boolean pulse = LOW;

    // Spin motor to stall
    while ( analogRead(ISENS) <=  maxCurrentCounts)
    {
        pulse = !pulse;
        digitalWrite(STEPPULSE, pulse);
        delayMicroseconds(intervalms);
    }
}

/* Heat up the substance */
void heatSubstance()
{
    const int TRefCounts = 750; // Corresponds to 130 deg C at platen
    const int tolCounts = 10;   // Corresponds to 5 deg C
    const int Thi  = TRefCounts + tolCounts; // Maximum limit
    const int Tlo  = TRefCounts - tolCounts; // Lower limit
    
    // Define timing variables
    unsigned tStart = millis();
    unsigned tEnd = tStart;

    // Define average temp
    int TAvg = avgTemp();

    digitalWrite(KHEATERS, HIGH); // Turn on the heaters

    // Wait for platen to heat up
    while ( !( (TAvg >= Tlo) && ( TAvg <= Thi) ) ) 
    {
        TAvg = avgTemp(); // Get current temp
    }

    digitalWrite(KHEATERS, LOW); // Turn the heaters off

    // Heat up substance for an hour with hysteresis control, run at 1Hz
    while ( (tEnd - tStart) <= 3600000 )
    {
        TAvg = avgTemp(); // Get current temp

        // Hysteresis control
        if ( TAvg > Thi )
        {
            digitalWrite(KHEATERS, LOW); // Turn the heaters off
        }
        else if (TAvg < Tlo)
        {
            digitalWrite(KHEATERS, HIGH); // Turn on the heaters
        } 

        // Wait 1 s to run at 1 Hz
        delay(1000);
        tEnd = millis();
    }

}


/* Unwind the motor to limit switch */
void unwindMotor()
{
    const int intervalms = 250;         // PWM frequency of (i.e. == RPM)

    digitalWrite(STEPENABLE, HIGH); // Write enable
    digitalWrite(STEPDIR, LOW);    // Set CCW motor direction
    digitalWrite(STEPPULSE, LOW);   // Set pulse low

    boolean pulse = LOW;

    // Spin motor to until limit switch is depressed
    while ( !digitalRead(LIMITSW) )
    {
        pulse = !pulse;
        digitalWrite(STEPPULSE, pulse);
        delayMicroseconds(intervalms);
    }
}


/* Flash LED and BUZZ*/
void buzzFlash()
{
    // Define timing variables
    unsigned tStart = millis();
    unsigned tEnd = tStart;

    // buzz and flash for 10 seconds
    while ( (tEnd - tStart) <= 10000 )
    {
        tone(BUZZ, 500);        // Pulse at X Hz
        pinPulse(LEDPIN, 500);  // Pulse LED at X ms
        tEnd = millis();
    }

    noTone(BUZZ); // turn buzzer off
    digitalWrite(LEDPIN, LOW); // turn off LED
}


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

    // Attach interupt to HWINT
    attachInterrupt(0, swISR, CHANGE);

    // Set LED to low
    digitalWrite(LEDPIN, LOW);
}


/* Main Routine */
void loop()
{
    // Check if user has decided to start
    if ( digitalRead(STARTSW) == HIGH )
    {
        // Keep LEDPIN high to notify user
        digitalWrite(LEDPIN, HIGH);

        // Wind the motor to stall
        windMotor();

        // Heat up the substance for X time
        heatSubstance();

        // Unwind the motor to start position
        unwindMotor();

        // Notify user that program has finished
        buzzFlash();
    }

}


/* SW interupt, occurs when HWINT goes low */
void swISR()
{
    // Turn off heaters
    digitalWrite(KHEATERS, LOW);

    // Turn off motor
    digitalWrite(STEPENABLE, LOW);
    digitalWrite(STEPPULSE, LOW);

    // Turn on fans
    digitalWrite(KFAN, LOW);
}

