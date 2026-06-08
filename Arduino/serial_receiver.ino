void setup() {
  // Start serial communication with the Raspberry Pi.
  Serial.begin(115200);
}

void loop() {
  // Read one complete command ending with a newline character.
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      Serial.print("Received: ");
      Serial.println(command);
    }
  }
}
