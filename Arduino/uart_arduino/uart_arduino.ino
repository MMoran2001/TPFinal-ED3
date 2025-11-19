#include <SoftwareSerial.h>

// Pines hacia la LPC (vista desde Arduino)
// RX (Arduino)  <- TXD2 (P0.10) de la LPC
// TX (Arduino)  -> RXD2 (P0.11) de la LPC
SoftwareSerial lpcSerial(10, 11);  // RX, TX

unsigned long lastSend = 0;

void setup() {
  // Serial para depurar con el PC
  Serial.begin(9600);
  // UART hacia la LPC
  lpcSerial.begin(9600);

  // Esperar a que el monitor serie esté listo (en placas con USB nativo, por si acaso)
  while (!Serial) {
    ; // no hacer nada
  }

  Serial.println("Arduino listo. Esperando datos de la LPC...");
  Serial.println("Cada 2s envio un mensaje a la LPC.");
}

void loop() {
  // 1) Leer lo que manda la LPC y mostrarlo en el monitor serie
  while (lpcSerial.available() > 0) {
    char c = (char) lpcSerial.read();
    Serial.print(c);  // lo ves como texto en el monitor serie
  }

  // 2) Enviar datos periódicos a la LPC (para que ella responda con "RX: x")
  unsigned long ahora = millis();
  if (ahora - lastSend >= 2000) { // cada 2 segundos
    lastSend = ahora;

    // Ejemplo sencillo: envío una letra que cambia
    static char testChar = 'A';
    if (testChar > 'Z') testChar = 'A';

    // Enviar un pequeño mensaje
    lpcSerial.print("ARDUINO: ");
    lpcSerial.print(testChar);
    lpcSerial.print("\r\n");

    Serial.print("Enviado a LPC -> ");
    Serial.println(testChar);

    testChar++;
  }

  // 3) (Opcional) Si escribís algo en el monitor serie del PC, se lo reenvío a la LPC
  if (Serial.available() > 0) {
    char c = (char) Serial.read();
    lpcSerial.write(c);  // esto va directo a la LPC
    Serial.print("PC->LPC: ");
    Serial.println(c);
  }
}
