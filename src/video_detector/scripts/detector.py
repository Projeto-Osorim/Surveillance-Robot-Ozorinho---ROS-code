# Imports

from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math 


# Object Classes

classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]


# Modelo de pesos - YOLOv8
model = YOLO("yolo-Weights/yolov8n.pt")


# Callback Function 
# função para subscrever os dados do Knect, e executar a detecção de imagem

def image_callback(msg):
    bridge = CvBridge() #Converte a ROS Image message em imagens capazes do OpenCV ler

    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") #Transforma a imagem recebida pelo ROS, de forma que o OpenCV entennda    
    
    results = model(img, stream=True) #Os resultados são: a imagem colocada no modelo de peso do yolov8

    # Coordenadas
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # converte para valores int

            # coloca as boxes na câmera
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # confidence
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->",confidence)

            # nome das classes
            cls = int(box.cls[0])
            print("Class name -->", classNames[cls])

            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

    cv2.imshow('Webcam', img) #Usado para mostrar os frames do vídeo
    cv2.waitKey(1) #permite que a janela contendo o vídeo fique aberta até que o usuário a feche 
    
    img_detection = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    img_pub.publish(img_detection)
    



# Main
    
rospy.init_node('image_subscriber', anonymous=True)

rospy.Subscriber('/camera/rgb/image_color', Image, image_callback)
img_pub = rospy.Publisher('/detectionImg_topic', Image, queue_size=10)

rospy.spin()