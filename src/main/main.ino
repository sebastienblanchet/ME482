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

PREPROCESSOR

**************************************************************************************************/

/* Hardware specifics */
#define MAXCOUNT     1023    // Max ADC counts
#define VREF         5       // Arduino reference voltage
#define RNOM         110000  // Thermistor resistance at 25 deg C
#define BCOEF        3950    // Thermistor B value for Steinhart equation
#define RSERIES      10000   // Pull up series resistor for thermistor sense
#define ISENSK       10      // Current sensor gain (i.e 1 V / 100mV/A)
#define ISENSOFF     25      // Current sensor offset (i.e 2.5 V / 100 mV/A)
#define TNOM         25      // Nominal temperature room temp 25 def C
#define STEPS        200     // Number of steps per rev
#define FWD          LOW     // Motor FWD direction TODO: confirm FWD DIR
#define BWD          HIGH    // Motor BWD direction TODO: confirm BWD DIR

/* Pinout based on schematic */
#define HWINT        2       // hardware interrupt (LOW == GOTO ISR)
#define KHEATERS     4       // heaters relay pin (HIGH == ON)
#define BUZZ         5       // buzzer PWM line (HIGH == ON)
#define LEDPIN       6       // controls LED in UI (HIGH == ON)
#define KFAN         7       // controls relay switch for fan (HIGH == ON)
#define STARTSW      8       // takes user input (LOW == START)
#define STEPENABLE   9       // enable the stepper motor (LOW == ON)
#define STEPDIR      10      // motor spin direction (HIGH == FWD, LOW == BWD) TODO: confirm directions
#define STEPPULSE    11      // PWM line for motor (HIGH == ON)
#define LIMITSW      12      // Limit switch for platen closed position (LOW == ON)
#define RETURNSW     13      // Limit switch for motor return position (LOW == ON)
#define THERM0       A0      // thermistor 1 reading
#define THERM1       A1      // thermistor 2 reading
#define ISENS        A2      // motor current reading


/*************************************************************************************************

GLOBALS

**************************************************************************************************/

/* Global variables */
// TODO: check if bool or timing should be global
// uint32_t tDelta;


/*************************************************************************************************

HELPER FUNCTIONS

**************************************************************************************************/

/* Get actual current sensor value */
// TODO: validate getIsensA
float getIsensA(void)
{
    // Map analog counts to voltage
    float voltIsens = map(analogRead(ISENS), 0, MAXCOUNT, 0, VREF);

    // Calculate measured current
    float current = (ISENSK * voltIsens) - ISENSOFF;

    return current;
}


/* Get actual temperature reading */
// TODO: validate getTempC
float getTempC(uint32_t countIn)
{
    // Convert counts to resistance
    float Rin = RSERIES / ((1023.0 / ( (float) countIn) - 1.0));

    // TODO: clean up equation
    // Tout = Rin / RNOM;     // (R/Ro)
    // Tout = log(Tout);                  // ln(R/Ro)
    // Tout /= BCOEF;                   // 1/B * ln(R/Ro)
    // Tout += 1.0 / (TNOM + 273.15); // + (1/To)
    // Tout = 1.0 / Tout;                 // Invert
    // Tout -= 273.15;                         // convert to C

    // Steinhart equation approximation
    float Tout =  (1.0 / ((1.0 / BCOEF) * log(Rin / RNOM)) + (1.0 / (TNOM + 273.15))) - 273.15;

    return  Tout;
}


/* Helper function to read avg temp count */
// TODO: validate avgTempC
float avgTempC(void)
{
    // Variable declarations
    uint32_t avgCounts;
    float avgTempC;

    // Read analog lines and average readings
    avgCounts += analogRead(THERM0);
    avgCounts += analogRead(THERM1);
    avgCounts /= 2;

    // Calculate temperature from Steinhart
    avgTempC = getTempC(avgCounts);

    // TODO: check if filter is required for avgTempC()

    return avgTempC;
}


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

/* Convert motor speed in RPM to pulse period in micro sec */
uint32_t getMotorTus(uint32_t motorSpeedRPM)
{
    uint32_t motorTus;

    motorTus = (60000000) / (STEPS * motorSpeedRPM);

    return motorTus;
}

/* Convert minutes to milliseconds */
uint32_t getMsFromMins(uint32_t minutes)
{
    return 60000 * minutes;
}


/*************************************************************************************************

SUBROUTINES

**************************************************************************************************/

/* Wind the motor to stall */
void windMotor(uint32_t windSpeedRPM, float ImaxA)
{
    // Write enable
    digitalWrite(STEPENABLE, LOW);

    // Set CW motor direction to close platens
    // TODO: confirm direction
    digitalWrite(STEPDIR, HIGH);

    // Set pulse low
    digitalWrite(STEPPULSE, LOW);
    bool pulse = LOW;

    // Get motor period from speed
    uint32_t windTus = getMotorTus(windSpeedRPM);

    // Spin motor to stall
    // TODO: confirm that this works, BW will likely be too high (noise)
    // TODO: confirm constant speed, may require some derating as torque increases
    // TODO: remove getIsensA() replace w/ sw
    // TODO: account for decarboxylation
    while ( getIsensA() <=  ImaxA )
    {
        pulse = !pulse;
        digitalWrite(STEPPULSE, pulse);
        delayMicroseconds(windTus);
    }

    // Turn off motor
    digitalWrite(STEPENABLE, HIGH);
}


/* Heat up the substance */
void heatSubstance(float TRefC, float THystC, uint32_t heatTmins)
{
    // TODO: finish function which converts temp to counts
    const float TmaxC  = TRefC + THystC; // Hysteresis max
    const float TminC  = TRefC - THystC; // Hysteresis min

    // Define timing variables
    uint32_t tStart = millis();
    uint32_t tEnd = tStart;
    uint32_t elapsed = tEnd - tStart;
    uint32_t heatTms = getMsFromMins(heatTmins);

    // Define average temp
    float TAvgC = avgTempC();

    // Turn on the heaters to begin warming up
    digitalWrite(KHEATERS, HIGH);

    // Wait for platen to heat up (i.e. not in hysteresis)
    while ( !( (TAvgC >= TminC) && ( TAvgC <= TmaxC) ) )
    {
        // Update the current temp
        TAvgC = avgTempC();
        delay(1000);
    }

    // Turn the heaters off
    digitalWrite(KHEATERS, LOW);

    // Heat up substance for an hour with hysteresis control, run at 1Hz
    while ( elapsed <= heatTmins )
    {
        // Get current temp
        TAvgC = avgTempC();

        // Hysteresis control
        if ( TAvgC > TmaxC )
        {
            // Turn the heaters off
            digitalWrite(KHEATERS, LOW);
        }
        else if (TAvgC < TminC)
        {
            // Turn on the heaters
            digitalWrite(KHEATERS, HIGH);
        }

        // Run control slower to avoid noise issues
        delay(1000);

        // Update elapsed time
        elapsed = millis() - tStart;
    }

}


/* Unwind the motor to limit switch */
void unwindMotor(uint32_t windSpeedRPM)
{
    // Write enable
    digitalWrite(STEPENABLE, LOW);

    // Set CCW motor direction to open platen
    // TODO: confirm direction
    digitalWrite(STEPDIR, LOW);

    // Set pulse low
    digitalWrite(STEPPULSE, LOW);
    bool pulse = LOW;

    // Get motor period from speed
    uint32_t windTus = getMotorTus(windSpeedRPM);

    // Spin motor to until limit switch is depressed
    // Recall, switch uses pull up resistor LOW == depressed
    while ( digitalRead(LIMITSW) )
    {
        pulse = !pulse;
        digitalWrite(STEPPULSE, pulse);
        delayMicroseconds(windTus);
    }

    // Turn off motor
    digitalWrite(STEPENABLE, HIGH);
}


/* Generic motor function */
void spinMotor(uint32_t windSpeedRPM, bool direction, uint32_t limitSW)
{
    // Write enable
    digitalWrite(STEPENABLE, LOW);

    // Set motor direction
    digitalWrite(STEPDIR, direction);

    // Set pulse low
    digitalWrite(STEPPULSE, LOW);
    bool pulse = LOW;

    // Get motor period from speed
    uint32_t windTus = getMotorTus(windSpeedRPM);

    // Spin motor to until specified limit switch is depressed
    while ( digitalRead(limitSW) )
    {
        pulse = !pulse;
        digitalWrite(STEPPULSE, pulse);
        delayMicroseconds(windTus);
    }

    // Turn off motor
    digitalWrite(STEPENABLE, HIGH);
}


/* Flash LED and BUZZ*/
void buzzFlash(uint32_t buzzFlashTmins)
{
    // Define timing variables
    uint32_t tStart = millis();
    uint32_t tEnd = tStart;
    uint32_t elapsed = tEnd - tStart;
    uint32_t buzzFlashTms = getMsFromMins(buzzFlashTmins);

    // buzz and flash for specified time
    while ( elapsed <= buzzFlashTms )
    {
        // Buzz at specific frequency
        tone(BUZZ, 500);

        // Flash LED
        pinPulse(LEDPIN, 500);

        // Update elapsed time
        elapsed = millis() - tStart;
    }

    // turn buzzer off
    noTone(BUZZ);
}


/*************************************************************************************************

MAIN ROUTINE

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

    // Attach interrupt to HWINT
    attachInterrupt(digitalPinToInterrupt(HWINT), swISR, HIGH);

    // Set LED to low
    digitalWrite(LEDPIN, LOW);

    // Set up serial port for debug
    // TODO: remove serial comm, will slow down control
    Serial.begin(9600);
}

/* Main Routine */
void loop()
{
    // Calibrations values
    // TODO: confirm cal locations
    const uint32_t windSpeedRPM = 100;     // Motor speed
    const float    ImaxA        = 3.0;     // Max current (i.e. represents stall)
    const float    TRefC        = 120.0;   // Reference platen temp
    const float    THystC       = 5.0;     // Hysteresis band
    const uint32_t heatTmins    = 5;       // Time to heat up substance in minutes
    const uint32_t buzzFlashTmins = 10000;   // Time period for buzz and flash

    // Check if user has decided to start
    if ( digitalRead(STARTSW) == LOW )
    {
        // 1. Keep LEDPIN high to notify user
        Serial.println("*********** STARTED      ***********");
        digitalWrite(LEDPIN, HIGH);

        // 2. Wind the motor to stall
        Serial.println("*********** WIND MOTOR   ***********");
        windMotor(windSpeedRPM, ImaxA - 1);
        // TODO: spinMotor(windSpeedRPM, FWD, LIMITSW);
        
        // TODO: Decarboxylation work on
        Serial.print("Decarboxylation ? (1/0): ");
        bool decarboxBool = Serial.read();

        if (decarboxBool)
        {
            heatSubstance(TRefC + 20, THystC, heatTmins / 5);
        }

        windMotor(windSpeedRPM, ImaxA);

        // 3. Heat up the substance for heatTmins
        Serial.println("*********** HEATING      ***********");
        heatSubstance(TRefC, THystC, heatTmins);

        // 4. Turn on fan
        Serial.println("*********** FAN          ***********");
        digitalWrite(KFAN, HIGH);

        // 5. Unwind the motor to start position
        Serial.println("*********** UNWIND MOTOR ***********");
        unwindMotor(windSpeedRPM);
        // TODO: spinMotor(windSpeedRPM, BWD, RETURNSW);

        // 6. Notify user that program has finished
        Serial.println("*********** BUZZ FLASH   ***********");
        buzzFlash(buzzFlashTmins);

        // 7. Turn fan off
        Serial.println("*********** END          ***********");
        digitalWrite(KFAN, LOW);

        // 8. Turn LEDPIN off to notify end of routine
        digitalWrite(LEDPIN, LOW);
    }
    else
    {
        // Notify of waiting
        Serial.println("*********** WAITING      ***********");
    }
}


/* SW interrupt, occurs when HWINT goes HIGH (i.e. SW open) */
// TODO: confirm swISR
void swISR()
{
    // Turn off heaters
    digitalWrite(KHEATERS, LOW);

    // Turn off motor
    digitalWrite(STEPENABLE, HIGH);
    digitalWrite(STEPPULSE, LOW);

    // Turn on fans
    digitalWrite(KFAN, HIGH);

    // TODO: figure out what it should do next
    // Notify user SW interrupt has occurred
    tone(BUZZ, 500);
    digitalWrite(LEDPIN, HIGH);
}

