'''
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import freenect  # Certifique-se de ter a biblioteca freenect instalada
import socket
import pickle
import struct

def publish_kinect_image():
    rospy.init_node('kinect_image_publisher', anonymous=True)
    rate = rospy.Rate(1)  # Defina a taxa de publicação conforme necessário
    
    # Função de callback para a captura de imagem do Kinect
    def kinect_callback(dev, data, timestamp):
        image_data = freenect.sync_get_video()[0]
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(image_data, encoding="bgr8")

        # Serializa a imagem em bytes
        image_bytes = pickle.dumps(image_msg)

        # Empacota o tamanho dos dados para envio
        size = struct.pack("!I", len(image_bytes))
        
        # Envia o tamanho dos dados e os dados da imagem via TCP
        client_socket.sendall(size + image_bytes)

    # Inicializa o Kinect
    freenect.init()
    dev = freenect.open_device(0)
    freenect.set_video_mode(dev, freenect.RESOLUTION_MEDIUM, freenect.VIDEO_RGB)
    freenect.set_depth_mode(dev, freenect.RESOLUTION_MEDIUM, freenect.DEPTH_REGISTERED)
    freenect.set_video_callback(dev, kinect_callback)
    freenect.start_video(dev)

    # Configuração da conexão TCP
    host = 'IP_DA_MAQUINA_DESTINO'  # Substitua pelo IP da máquina de destino
    port = 12345  # Substitua pela porta desejada
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))

    while not rospy.is_shutdown():
        # Aguarda a taxa de publicação
        rate.sleep()

    # Encerra o Kinect e a conexão TCP quando o nó é encerrado
    freenect.stop_video(dev)
    freenect.close_device(dev)
    client_socket.close()
    freenect.shutdown()

if __name__ == '__main__':
    try:
        publish_kinect_image()
    except rospy.ROSInterruptException:
        pass
'''