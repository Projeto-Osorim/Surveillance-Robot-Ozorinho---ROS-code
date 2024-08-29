import rospy
from std_msgs.msg import String
import serial

# Define a porta serial
#serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=0.1)
serial_arduino = rospy.Publisher("/serial_arduino", String, queue_size=10)

'''
Abrir a porta serial : 

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
'''
def callback(msg):
    message = msg.data
    #print(message)
    #serial_port.write(message.encode('utf-8'))
    serial_arduino.publish(message)

def serial_subscriber():
    rospy.init_node('serial_subscriber', anonymous=True)
    rospy.Subscriber("/serial_command", String, callback)

    # Espera por novas mensagens
    rospy.spin()

    '''if serial_port.is_open:
        serial_port.close()'''

if __name__ == '__main__':
    try:
        serial_subscriber()
    except rospy.ROSInterruptException:
        pass
