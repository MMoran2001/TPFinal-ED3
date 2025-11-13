// Arduino
// Conectar RX<-TXD2 (P0.10), TX->RXD2 (P0.11), GND-GND
// Ajusta 9600 si cambiaste BAUD_RATE en el LPC
void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println("Arduino listo @ 9600");
}

unsigned long t0 = 0;
int n = 0;

void loop() {
  // Echo de lo que llegue desde el LPC
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    Serial.print("ACK: ");
    Serial.println(line);
  }

  // Enviar algo periódicamente al LPC
  if (millis() - t0 >= 1000) {
    t0 = millis();
    Serial.print("hello ");
    Serial.println(n++);
  }
}