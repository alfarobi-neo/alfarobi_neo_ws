#!/usr/bin python3
import rospy
from std_msgs.msg import String

button = str()
count = 1
def callback(data):
    global button, count
    button = data.data
    if(button != "None"):
        count += 1
        rospy.loginfo(rospy.get_caller_id() + "FUNGSI A %s", data.data)
        
    else:
        count += 1
        rospy.loginfo("FUNGSI B : %d", count)
        
    
def listener():
    global button
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/arduino_controller/button", String, callback)
    
    # if(button != "None"):
    print("TEST")
        

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()