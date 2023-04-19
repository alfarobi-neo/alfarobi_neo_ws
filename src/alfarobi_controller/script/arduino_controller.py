import serial
import re

# initialize serial communication
ser = serial.Serial('/dev/ttyACM0', 9600) # change 'COM3' to the appropriate port and 9600 to the appropriate baud rate

# read and process data from Arduino
while True:
    data = ser.readline().decode().rstrip() # read one line of data from the serial port, decode it from bytes to string and remove newline character
    match = re.search(r'U(\d+)G(\d+)M(\d+)', data) # use regular expressions to match the numbers after 'U', 'G', and 'M'
    if match: # if there is a match
        u_value = int(match.group(1)) # extract the first numeric value and convert it to an integer
        g_value = int(match.group(2)) # extract the second numeric value and convert it to an integer
        m_value = int(match.group(3)) # extract the third numeric value and convert it to an integer
        print(u_value, g_value, m_value) # process the values as needed
