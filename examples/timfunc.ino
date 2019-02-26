
void myFunction()
{
    // Do stuff
}

void setup() {

    Serial.begin(9600);

}

void loop() {

    unsigned long start = millis();

    // Call to your function
    myFunction();

    // Compute the time it took
    unsigned long delta = millis() - start;
}
