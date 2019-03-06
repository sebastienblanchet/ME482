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
#define STEPS    200     // Number of steps per rev

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


/* Helper function for pin pulse */
void pinPulse(int pinNum, int delayMs)
{
    // Set pin high and wait
    digitalWrite(pinNum, HIGH);
    delay(delayMs);

    // Return pin to low and wait
    digitalWrite(pinNum, LOW);
    delay(delayMs);
}


/* Flash LED and BUZZ*/
void buzzFlash(uint32_t buzzFlashTms)
{
    // Define timing variables
    uint32_t tStart = millis();
    uint32_t tEnd = tStart;
    uint32_t elapsed = tEnd - tStart;

    // buzz and flash for specified time
    while ( elapsed <= buzzFlashTms )
    {
        // TODO: specify time and pulse rate
        // Pulse at 5 Hz
        tone(BUZZ, 10);
        // Pulse LED at X ms
        pinPulse(LEDPIN, 250);

        // Update elapsed time
        elapsed = millis() - tStart;
    }

    // turn buzzer off
    noTone(BUZZ);
}


/*************************************************************************************************
**************************************************************************************************/

/* Setup the Arduino hardware */
void setup()
{
    // Define digital pins
    // 
    int run = 0;
    pinMode(HWINT,      INPUT );
    pinMode(KHEATERS,   OUTPUT);
    pinMode(BUZZ,       OUTPUT);
    pinMode(LEDPIN,     OUTPUT);
    pinMode(KFAN,       OUTPUT);
    pinMode(STARTSW,    INPUT );
    pinMode(STEPENABLE, OUTPUT);
    pinMode(STEPDIR,    OUTPUT);
    pinMode(STEPPULSE,  OUTPUT);

    // Set LED to low
    digitalWrite(LEDPIN, LOW);

    // Set up serial port for debug
    Serial.begin(9600);

    Serial.println("*********** WAITING ***********");

    // while (digitalRead(STARTSW) == LOW) {}
}

// Calibrations values
const uint32_t windSpeedRPM = 100;   // Motor speed
const uint32_t ImaxA = 3.0;          // Max current (i.e. represents stall)
const float TRefC = 120.0;           // Reference platen temp
const float THystC = 5.0;            // Hysteresis band
const uint32_t heatTms = 3600000;    // Time to heat up substance in ms
const uint32_t buzzFlashTms = 5000; // Time period for buzz and flash

/* Main Routine */
void loop()
{
    if (digitalRead(STARTSW) == LOW)
    {    
        Serial.println("*********** STARTED ***********");
        buzzFlash(buzzFlashTms);
    }
    else
    {
        Serial.println("*********** WAITING ***********");
        delay(500);
    }
}
