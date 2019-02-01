
void myFunction()
{
    // Do stuff
}

void setup() {

    Serial.begin(9600);

}

void loop() {
    // https://www.arduino.cc/reference/en/language/functions/time/micros/
    unsigned long start = micros();
    // Call to your function
    myFunction();
    // Compute the time it took
    // unsigned long end = micros();
    unsigned long delta = micros() - start;
    // unsigned long delta = end - start;
    Serial.println(delta)
}
