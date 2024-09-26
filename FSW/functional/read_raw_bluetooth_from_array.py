import serial
import rospy
from typing import Generator
from ..config.topic_names import RAW_BLUETOOTH
from ..config.constants import RGV_ID
from rosardvarc.msg import BluetoothAzimuthElevation

# SERIAL PORT READ LINE FUNCTION
def read_lines(s: serial.Serial, sep: bytes = b"\r\n\r\n") -> Generator[bytes, None, None]: #/r/n/r/n: carriage return, readline, carriage return, readline (what the array outputs to denote new line)
    buffer = b"" # initialise buffer to store line from serial port
    while True:
        buffer += s.read() # read only when UTF-8 character is available on the port
        while sep in buffer: # triggers when /r/n/r/n is detected
            line, _, buffer = buffer.partition(sep) # store buffer in variable line, variable line changes every time new line is complete
            yield line

# Make ROS node for bluetooth data reading
rospy.init_node("read_raw_bluetooth")

# Create publisher for raw bluetooth
bluetooth_pub = rospy.Publisher(RAW_BLUETOOTH, BluetoothAzimuthElevation, queue_size=1)

# Create message that will be updated and sent each time a new raw measurement is received
msg = BluetoothAzimuthElevation()

# Call up serial port
s = serial.Serial("/dev/ttyUSB0", 115200) # CHANGE TO COMXX PORT FOR WINDOWS, baud rate stays the same

for line in read_lines(s): # run every time a new line is detected
    # Exit early if ROS is shutdown
    if rospy.is_shutdown():
        quit()
    
    # Decode the line
    decoded = line.decode('utf-8') # decode bytes into characters
    splitted = decoded.split(',') # split string by comma character
    if line[6:9] == b'D77': # Read ID of beacon 1
        b_id = RGV_ID.RGV1 # Save 1 to b_id for beacon 1
    else: # NEED TO WRITE ELSE STATEMENT FOR SECOND BEACON
        b_id = RGV_ID.RGV2 # Save 2 to b_id for beacon 2
    ang1 = int(splitted[2]) # Parse angle 1
    ang2 = int(splitted[3]) # Parse angle 2
    
    # Fill out message
    msg.timestamp = rospy.Time.now()
    msg.rgv_id = b_id
    msg.azimuth = ang1
    msg.elevation = ang2
    
    # Send message
    bluetooth_pub.publish(msg)