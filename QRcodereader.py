# Luis Nolasco Ramirez
#
# Codigo correspondiente a la lectura de codigos QR con la camara del robot,
# En caso de que el topic que da inicio al codigo este activo 
# Se recibe la informacion de la camara del robot, se procesa y analiza en busqueda
# de codigos QR y se almacena la informacion recibida de los mismos
#

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2, cv_bridge
import pyzbar.pyzbar as pyzbar

# Clase QR utilizada para la inicializacion del suscriptor al topic de la camara y la definicion de las funciones necesarias
class QR:
  # Inicializador de clase
  def __init__(self):
    #Se inicia el objeto cvBridge para transformar el mensaje de tipo Image a tipo bgr8
    self.bridge = cv_bridge.CvBridge()
    #Se inicia el suscriptor al topic de la camara
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    #Se inicia el suscriptor al topic qrcode/start
    self.image_sub = rospy.Subscriber('qrcode/start',String,self.start_callback)
    #Se inicia la variable de clase start con el valor "stop"
    self.start = "stop"

  # callback de inicio de lectura de la camara
  def start_callback(self,msg):
    #Convertir el mensaje en string
    data = str(msg)
    #Almacenar solo la parte del string que contiene la informacion
    data_ = data[7:-1]
    print(data_)
    #Asignar el valor leido a la variable de clase start
    self.start = data_

  # callback de cada imagen recibida de la camara
  def image_callback(self, msg):
    #Transformar el mensaje de tipo Image a tipo bgr8 para trabajar con opencv
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    #Inicia el publisher en el topic qrcode/info
    pub_qrinfo = rospy.Publisher('qrcode/info', String, queue_size=5) #Se inicia la variable para publicar en el topic correspondiente
    #Fuente a usar para mostrar la informacion del paquete sobre la imagen
    font = cv2.FONT_HERSHEY_PLAIN
    #Funcion de la libreria pyzbar para obtener la informacion de todos los qr en la imagen
    decodedObjects = pyzbar.decode(image)

    #Para cada qr en la imagen
    for obj in decodedObjects:
        #Se transforma la informacion en un string que solo almacena el nombre del paquete
        data_str = str(obj.data)
        data = data_str[2:len(data_str)-1]
        print("Data ", data)
        #Mostrar el nombre del paquete sobre la imagen
        cv2.putText(image, str(obj.data), (50, 50), font, 2,
                    (255, 0, 0), 3)
    
    #Si no hay QRs en la imagen o la variable de clase start almacena "stop"
    if len(decodedObjects)==0 or self.start=="stop":
        print("No QR")
    #Si hay QRs en la imagen y la variable de clase start almacena "start" se publica la informacion del QR en el topic qrcode/info
    if len(decodedObjects)!=0 and self.start=="start":
        print("publish ",data)
        pub_qrinfo.publish(data)

#Se inicia el nodo QRreader
rospy.init_node('QRreader')
#Se crea la variable de clase
qrcode = QR()
rospy.spin()
