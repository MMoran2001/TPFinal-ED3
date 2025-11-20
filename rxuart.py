import serial
import time

# Configurar puerto serie (AJUSTÁ BAUDRATE si es necesario)
ser = serial.Serial(
    port="COM9",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1     # 1 segundo de espera por dato
)

print("Esperando datos en COM9...\n")

datos_recibidos = []

try:
    while True:
        if ser.in_waiting > 0:           # Si hay datos en el buffer
            byte = ser.read(1)           # Leer 1 byte
            valor = int.from_bytes(byte, "little")
            datos_recibidos.append(valor)

            print(f"Recibido: {valor}")

except KeyboardInterrupt:
    print("\n\nRecepción detenida por el usuario.")

finally:
    ser.close()
    print("Puerto COM9 cerrado.")

    
