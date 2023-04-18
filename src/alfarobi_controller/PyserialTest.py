 #!/usr/bin python3
import serial.tools.list_ports
import rospy
import rospkg
import numpy as np
import yaml
import threading
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def initialize():
    global port_name
    rospy.init_node('arduino_controller_node')

    rospy.set_param('/arduino_controller/portname', 'ttyACM0')
    rospy.set_param('/arduino_controller/baudrate', 1000000)
    rospy.set_param('/arduino_controller/waitTimeout', 500)
    rospy.set_param('/arduino_controller/broadcast', True)

    port_name = rospy.get_param('/arduino_controller/portname')
    baud_rate = rospy.get_param('/arduino_controller/baudrate')
    wait_timeout = rospy.get_param('/arduino_controller/waitTimeout')
    broadcast = rospy.get_param('/arduino_controller/broadcast')

    rospy.loginfo(f'Selected Port: {port_name} Rate: {baud_rate}')
    port_name = port_name

    deviceCheck()

    default_path = rospkg.RosPack().get_path('alfarobi_controller') + "/GlobalConfig.yaml"
    loadButton(default_path)

def deviceCheck():
    global port_name, currentPortName,currentPortNameChanged
    ports = serial.tools.list_ports.comports()
    foundPort = False
    for p in ports:
        if port_name == p.device and hex(p.pid) != '0x6014':
            print("Found Port: ", p.device)
            devicedata = p
            # debugDevice()
            dev_connected = True
            foundPort = True
            break
        elif hex(p.pid) != '0x6014':
            port_name = p.device
            devicedata = p
            print("Found Suitable Arduino Port: ", p.device)
            dev_connected = True
            foundPort = True
            break

    mutex = threading.Lock()
    mutex.acquire()
    if currentPortName != port_name:
        currentPortName = port_name
        currentPortNameChanged = True
        dev_connected = False

    if foundPort:
        threading.Thread().start()
    
    mutex.release()


button_mapping = {}

def loadButton(path):
    with open(path, 'r') as stream:
        try:
            # load yaml
            doc = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print(e)
            return

    # parse button_setting
    button_mode = doc.get("button_setting")
    for button, mode in button_mode.items():
        index = int(mode)
        mode = str(button)

        button_mapping[index] = mode

currentPortNameChanged = False
currentPortName = ""
initialize()
  
reconnect = False
        
imu_pub = rospy.Publisher('/arduino_controller/imu', Imu, queue_size=10)
button_pub = rospy.Publisher('/arduino_controller/button', String, queue_size=10)

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

portsList = []

for onePort in ports:
    portsList.append(str(onePort))
    print(str(onePort))

serialInst.baudrate = 1000000
serialInst.port = "/dev/ttyACM0"
serialInst.open()

count = 1
data = ""

roll = 0
pitch = 0
yaw = 0
gyroRoll = 0
gyroPitch = 0
gyroYaw = 0
accelX = 0
accelY = 0
accelZ = 0
button = 0
prev_button = 999

orientiation = Quaternion()

imu = Imu()
button_str = String()
publish_none = False
# rate = rospy.Rate(1000000)

# while True:
while not rospy.is_shutdown():
    for line in serialInst.read():
        char = chr(line)
        data += char
        if char == 'A':
            print(data)
            try:
                pos_U = data.rfind("U")
                pos_G = data.rfind("G")
                pos_M = data.rfind("M")
                pos_D = data.rfind("D")
                pos_I = data.rfind("I")
                pos_Y = data.rfind("Y")
                pos_P = data.rfind("P")
                pos_C = data.rfind("C")
                pos_T = data.rfind("T")
                pos_B = data.rfind("B")
                pos_A = data.rfind("A")
                roll_str = data[pos_U + 1 : pos_G]
                pitch_str = data[pos_G + 1 : pos_M]
                yaw_str = data[pos_M + 1 : pos_D]
                gyroRoll_str = data[pos_D + 1 : pos_I]
                gyroPitch_str = data[pos_I + 1 : pos_Y]
                gyroYaw_str = data[pos_Y + 1 : pos_P]
                accelX_str = data[pos_P + 1 : pos_C]
                accelY_str = data[pos_C + 1 : pos_T]
                accelZ_str = data[pos_T + 1 : pos_B]
                roll = float(roll_str)
                pitch = float(pitch_str)
                yaw = float(yaw_str)
                gyroRoll = float(gyroRoll_str)
                gyroPitch = float(gyroPitch_str)
                gyroYaw = float(gyroYaw_str)
                accelX = float(accelX_str)
                accelY = float(accelY_str)
                accelZ = float(accelZ_str)
                button = int(data[pos_B + 1 : pos_A])
            except ValueError:
                rospy.loginfo("Arduino imu data error")
            print(roll)
            print(pitch)
            print(yaw)
            print(gyroRoll)
            print(gyroPitch)
            print(gyroYaw)
            print(accelX)
            print(accelY)
            print(accelZ)
            print(button)
            print("===========================================================================")
            data = ""
            
    quarternion = get_quaternion_from_euler(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
    gyro = [np.deg2rad(gyroRoll), np.deg2rad(gyroPitch), np.deg2rad(gyroYaw)]
    accel = [accelX, accelY, accelZ]
    
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = "imu_link"
    
    imu.orientation.x = quarternion[0]
    imu.orientation.y = quarternion[1]
    imu.orientation.z = quarternion[2]
    imu.orientation.w = quarternion[3]
    
    imu.angular_velocity.x = gyro[0]
    imu.angular_velocity.y = gyro[1]
    imu.angular_velocity.z = gyro[2]
    
    imu.linear_acceleration.x = accel[0]
    imu.linear_acceleration.y = accel[1]
    imu.linear_acceleration.z = accel[2]
    
    imu_pub.publish(imu)
    
    if button != prev_button:
        if button in button_mapping:
            data_ = button_mapping[button]
            button_str.data = data_
            prev_button = button
            button_pub.publish(button_str)
            publish_none = True
    elif (prev_button == button):
        if(publish_none):
            button_str.data = "None"
            button_pub.publish(button_str)
            publish_none = False
    
    
    

      
    # print(imu.orientation, "\n")
    # print(imu.angular_velocity, "\n")
    # print(imu.linear_acceleration, "\n")
    # print(button)
    # rate.sleep()
serialInst.close()