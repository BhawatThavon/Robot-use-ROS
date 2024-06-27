import serial

# Initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)  # Update the port and baud rate

while True:
    # Read a line from serial
    line = ser.readline().decode().strip()
    
    # Check if the line starts with "AV" to indicate angular velocity data
    if line.startswith("AV"):
        angular_velocity = float(line.split()[1])  # Extract angular velocity value
        # Do something with the angular velocity, such as print it
        print("Received Angular Velocity:", angular_velocity)

