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
// uint32_t tDelta;


/* Helper function for pin pulse */
void pinPulse(int pinNum, int delayMs)
{
    digitalWrite(pinNum, HIGH);
    delay(delayMs);
    digitalWrite(pinNum, LOW);
    delay(delayMs);
}


/* Get actual temperature reading */
float get_steinhart(uint32_t countIn) {

    const float Rnom = 110000.0;
    const float B = 3950.0;
    const float Tnom = 25.0;
    const float Rseries = 10000;

    float Tout;

    countIn = (float) (1023 / countIn - 1);

    float Rin = Rseries / countIn;

    Tout = Rin / Rnom;     // (R/Ro)
    Tout = log(Tout);                  // ln(R/Ro)
    Tout /= B;                   // 1/B * ln(R/Ro)
    Tout += 1.0 / (Tnom + 273.15); // + (1/To)
    Tout = 1.0 / Tout;                 // Invert
    Tout -= 273.15;                         // convert to C

    return  Tout;
}


/* Helper function to read avg temp count */
uint32_t avgTemp()
{
    uint32_t avgTemp;

    avgTemp += analogRead(THERM0);
    avgTemp += analogRead(THERM1);
    avgTemp /= 2;

    return avgTemp;
}


/* Wind the motor to stall */
void windMotor(uint32_t windTus, uint32_t Imax)
{
    // Corresponds to 2.5 A from motor
    // const uint32_t maxCurrentCounts = 200;
    // TODO: write a function which converts motor speed to us

    // Write enable
    digitalWrite(STEPENABLE, HIGH);
    // Set CW motor direction
    // TODO: confirm direction
    digitalWrite(STEPDIR, HIGH);
    // Set pulse low
    digitalWrite(STEPPULSE, LOW);

    boolean pulse = LOW;

    // Spin motor to stall
    while ( analogRead(ISENS) <=  Imax)
    {
        pulse = !pulse;
        digitalWrite(STEPPULSE, pulse);
        delayMicroseconds(windTus);
    }
}


/* Heat up the substance */
void heatSubstance(uint32_t heatTime)
{
    // TODO: finish function which converts temp to counts
    const uint32_t TRefCounts = 750; // Corresponds to 130 deg C at platen
    const uint32_t tolCounts = 10;   // Corresponds to 5 deg C
    const uint32_t Thi  = TRefCounts + tolCounts; // Maximum limit
    const uint32_t Tlo  = TRefCounts - tolCounts; // Lower limit

    // Define timing variables
    uint32_t tStart = millis();
    uint32_t tEnd = tStart;

    // Define average temp
    uint32_t TAvg = avgTemp();

    // Turn on the heaters to begin warming up
    digitalWrite(KHEATERS, HIGH);

    // Wait for platen to heat up (i.e. not in hysteresis)
    while ( !( (TAvg >= Tlo) && ( TAvg <= Thi) ) )
    {
        // Update the current temp
        TAvg = avgTemp();
    }

    // Turn the heaters off
    digitalWrite(KHEATERS, LOW);

    // Heat up substance for an hour with hysteresis control, run at 1Hz
    while ( (tEnd - tStart) <= heatTime )
    {
        // Get current temp
        TAvg = avgTemp();

        // Hysteresis control
        if ( TAvg > Thi )
        {
            // Turn the heaters off
            digitalWrite(KHEATERS, LOW);
        }
        else if (TAvg < Tlo)
        {
            // Turn on the heaters
            digitalWrite(KHEATERS, HIGH);
        }

        // Run control slower to avoid noise issues
        delay(1000);

        // Update elapsed time
        tEnd = millis();
    }

    // Turn on fan
    digitalWrite(KFAN, HIGH);
}


/* Unwind the motor to limit switch */
void unwindMotor(uint32_t windTus)
{
    // Write enable
    digitalWrite(STEPENABLE, HIGH);
    // Set CCW motor direction
    digitalWrite(STEPDIR, LOW);
    // Set pulse low
    digitalWrite(STEPPULSE, LOW);

    boolean pulse = LOW;

    // Spin motor to until limit switch is depressed
    while ( !digitalRead(LIMITSW) )
    {
        pulse = !pulse;
        digitalWrite(STEPPULSE, pulse);
        delayMicroseconds(windTus);
    }
}


/* Flash LED and BUZZ*/
void buzzFlash(uint32_t buzzFlashTime)
{
    // Define timing variables
    uint32_t tStart = millis();
    uint32_t tEnd = tStart;

    // buzz and flash for 10 seconds
    while ( (tEnd - tStart) <= buzzFlashTime )
    {
        // TODO: specify time and pulse rate
        // Pulse at X Hz
        tone(BUZZ, 500);
        // Pulse LED at X ms
        pinPulse(LEDPIN, 500);
        tEnd = millis();
    }

    // turn buzzer off
    noTone(BUZZ);
    // turn off LED
    digitalWrite(LEDPIN, LOW);
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
    // Calibrations
    const uint32_t buzzFlashTime = 10000;   // Time period for buzz and flash
    const uint32_t heatTime = 3600000;      // Time to heat up substance in ms
    const uint32_t windTus = 250;           // Motor speed
    const uint32_t Imax = 500;              // Max current counts


    // Check if user has decided to start
    if ( digitalRead(STARTSW) == HIGH )
    {
        // Keep LEDPIN high to notify user
        digitalWrite(LEDPIN, HIGH);

        // Wind the motor to stall
        windMotor(windTus, Imax);

        // Heat up the substance for X time
        heatSubstance(heatTime);

        // Unwind the motor to start position
        unwindMotor(windTus);

        // Notify user that program has finished
        buzzFlash(buzzFlashTime);
    }

}


/* SW interrupt, occurs when HWINT goes low */
void swISR()
{
    // Turn off heaters
    digitalWrite(KHEATERS, LOW);

    // Turn off motor
    digitalWrite(STEPENABLE, LOW);
    digitalWrite(STEPPULSE, LOW);

    // Turn on fans
    digitalWrite(KFAN, LOW);

    // TODO: figure out what it should do next
    // Buzz to notify reboot?
}

