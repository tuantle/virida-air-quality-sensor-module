
/*
Uses a FOR loop for data and prints a number in various formats.
*/

void setup() {
    Serial1.begin(9600);      // open the serial port at 9600 bps:
    delay(1000);
}

void loop() {
    // send data only when you receive data:
    if (Serial1.available()) {
            // read the incoming byte:
            char incomingByte = Serial1.read();

            // say what you got:
            Serial.print("I received: "); Serial.println(incomingByte);
    }
}
