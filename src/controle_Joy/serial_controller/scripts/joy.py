import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

serial_command_pub = rospy.Publisher("/serial_command", String, queue_size=10)

'''
Ligar o Nó do Joy Stick

rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node
'''

def callback(msg):
    try:
        if msg.axes[7] == 1:
            message = 'W'
        elif msg.axes[7] == -1:
            message = 'S'
        elif msg.axes[6] == 1:
            message = 'A'
        elif msg.axes[6] == -1:
            message = 'D'
        elif msg.buttons[0] == 1:
            message = 'P'
        elif msg.buttons[1] == 1:
            message = 'T'
        else:
            message = ' '
    except IndexError:
        if msg.axes[1] == 1:
            message = 'W'
        elif msg.axes[1] == -1:
            message = 'S'
        elif msg.axes[0] == 1:
            message = 'A'
        elif msg.axes[0] == -1:
            message = 'D'
        elif msg.buttons[2] == 1:
            message = 'P'
        else:
            message = ' '


    serial_command_pub.publish(message)
    print(message)

def joystick_control():
    rospy.init_node('joystick_control', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        joystick_control()
    except rospy.ROSInterruptException:
        pass
