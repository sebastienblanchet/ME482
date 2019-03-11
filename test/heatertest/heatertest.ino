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
#define RSERIES      100000   // Pull up series resistor for thermistor sense
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

/* Get actual temperature reading */
// TODO: validate getTempC
float getTempC(uint32_t countIn)
{
    // Convert counts to resistance
    float Rin = RSERIES / (1023.0/ countIn - 1.0);

    // Steinhart equation approximation
    float ToutC =  (1.0 / (((1.0 / BCOEF) * log(Rin / RNOM)) + (1.0 / (TNOM + 273.15)))) - 273.15;

    return  ToutC;
}


/* Helper function to read avg temp count */
// TODO: validate avgTempC
float avgTempC()
{
    // Variable declarations
    uint32_t avgCounts;
    float avgTempC;

    // Read analog lines and average readings
    uint32_t count1 = analogRead(THERM0);
    uint32_t count2 = analogRead(THERM1);
    
    Serial.println(getTempC(count1));
    Serial.println(getTempC(count2));

    avgTempC += getTempC(count1);
    avgTempC += getTempC(count2);
    avgTempC /= 2;

    Serial.println(TAvgC);

    return avgTempC;
}

/* Convert minutes to milliseconds */
uint32_t convertMsToMin(uint32_t minutes)
{
    return 60000 * minutes;
}


/*************************************************************************************************

SUBROUTINES

**************************************************************************************************/

/* Heat up the substance */
void heatSubstance(float TRefC, float THystC, uint32_t heatTmins)
{
    // TODO: finish function which converts temp to counts
    const float TmaxC  = TRefC + THystC; // Hysteresis max
    const float TminC  = TRefC - THystC; // Hysteresis min

    // Define average temp
    Serial.print("Initial Temperature [C] ");
    float TAvgC = avgTempC();

    // Turn on the heaters to begin warming up
    digitalWrite(KHEATERS, LOW);
    Serial.println("HEATERS ON");

    // Wait for platen to heat up (i.e. not in hysteresis)
    Serial.println(TmaxC);

    while ( (TAvgC < TmaxC) )
    {
        // Update the current temp
        Serial.println("Waiting for Platen to heat");
        TAvgC = avgTempC();
        delay(1000);
    }

    Serial.print("Platen heated to Temperature [C] ");
    Serial.println(TAvgC);

    // Turn the heaters off
    digitalWrite(KHEATERS, HIGH);
    Serial.println("HEATERS OFF");

    // Define timing variables
    uint32_t tStart = millis();
    uint32_t tEnd = tStart;
    uint32_t elapsed = tEnd - tStart;
    uint32_t heatTms = convertMsToMin(heatTmins);

    // Heat up substance for an hour with hysteresis control, run at 1Hz
    while ( elapsed <= heatTms )
    {
        // Get current temp
        Serial.println("Main heating loop");
        TAvgC = avgTempC();

        // Hysteresis control
        if ( TAvgC > TmaxC )
        {
            // Turn the heaters off
            digitalWrite(KHEATERS, HIGH);
            Serial.println("HEATERS OFF");
        }
        else if (TAvgC < TminC)
        {
            // Turn on the heaters
            digitalWrite(KHEATERS, LOW);
            Serial.println("HEATERS ON");
        }

        // Run control slower to avoid noise issues
        delay(1000);

        // Update elapsed time
        elapsed = millis() - tStart;

        Serial.print("Elapsed [s]: ");
        Serial.println(elapsed / 1000);
    }

    // Turn off the heaters
    digitalWrite(KHEATERS, HIGH);
    digitalWrite(KFAN, LOW);

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

    // Set LED to low
    digitalWrite(LEDPIN, LOW);

    // Turn off heaters and fans
    digitalWrite(KHEATERS, HIGH);
    digitalWrite(KFAN, HIGH);

    // Set up serial port for debug
    // TODO: remove serial comm, will slow down control
    Serial.begin(9600);
}

/* Main Routine */
void loop()
{
    // Calibrations values
    const float    TRefC        = 110.0;   // Reference platen temp
    const float    THystC       = 2.0;     // Hysteresis band
    const uint32_t heatTmins    = 2;       // Time to heat up substance in minutes

        // Check if user has decided to start
    if ( digitalRead(STARTSW) == LOW )
    {
        // 3. Heat up the substance for heatTmins
        Serial.println("*********** STARTING      ***********");
        heatSubstance(TRefC, THystC, heatTmins);
    }
    else
    {
        Serial.println("*********** WAITING      ***********");
    }
}