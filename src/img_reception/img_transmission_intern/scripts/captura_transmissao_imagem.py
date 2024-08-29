
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def publish_image():
    rospy.init_node('image_publisher', anonymous=True)
    rate = rospy.Rate(120)  # Defina a taxa de publicação conforme necessário
    
    # Inicialize a câmera (substitua pela sua lógica de captura de imagem)
    cap = cv2.VideoCapture(2)

    # Inicialize o publicador de imagem
    image_pub = rospy.Publisher('/image_topic', Image, queue_size=10)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # Captura a imagem da câmera
        ret, frame = cap.read()

        # Converte a imagem para a mensagem Image
        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publica a imagem
        image_pub.publish(image_msg)

        rate.sleep()

    # Libera a câmera quando o nó é encerrado
    cap.release()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass