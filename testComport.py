import serial

# Replace 'COM3' with the appropriate port for your system
# On Linux/Mac, it might be something like '/dev/ttyUSB0'
port = 'COM4'

# Open the serial port
ser = serial.Serial( port = port, baudrate = 9600, timeout=0)

# Read data from the device
while True:
    if ser.in_waiting > 0:
        data = ser.read(33)
        print(data)

# Close the serial port
ser.close()
