import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        bridge = CvBridge()
        # Converte a mensagem Image de volta para a imagem OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Exibe a imagem (substitua esta parte pelo seu processamento desejado)
        cv2.imshow("Received Image", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(e)

def image_subscriber():
    rospy.init_node('image_subscriber', anonymous=True)

    # Subscreva o tópico de imagem
    rospy.Subscriber('/image_topic', Image, image_callback)

    # Mantenha o programa em execução
    rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass