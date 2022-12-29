#Luis Nolasco Ramirez
#
# Codigo que permite al robot revisar una serie de puntos dados en una nave industrial y comprobar si en dicha
# posicion hay o no un codigo QR (que representa a un paquete). Ademas almacena en un archivo de texto
# cuales han sido las lecturas al pasar por cada punto de forma que se puede conocer si en cada una de las 
# iteraciones del robot habia o no habia un paquete en ese punto.
# 
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
import sys
from geometry_msgs.msg import Twist
import time

import cv2, cv_bridge
from sensor_msgs.msg import Image
import numpy as np
import pyzbar.pyzbar as pyzbar


class ClienteMoveBase:
    def __init__(self):
        #Variable de clase auxiliar que indica la existencia o no de codigo QR
        self.qrcode = False
        #Tiempo inicial
        self.initial_time = time.time()
        #Iniciar los diferentes suscriptores y publicadores
        self.startcamera = rospy.Publisher('/qrcode/start', String, queue_size=5)
        self.pub_velocity = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5) #Se inicia la variable para publicar en el topic correspondiente
        self.sub_info = rospy.Subscriber("qrcode/info", String, self.qrfound)
        #Iniciar la accion
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

    #Callback del topic comando para detener la accion
    def escucharComando(self,datos):
        if datos.data == "STOP":
            print("PARANDO!!!")
            self.client.cancel_goal()
    
    #Funcion para ir a la posicion inicial, necesario definir la posicion inicial previamente
    def home(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = -1.5        #Coordenadas de HOME position   
        goal.target_pose.pose.position.y = -0.5
        goal.target_pose.pose.orientation.w = 1.0
        rospy.Subscriber("comando", String, self.escucharComando)
        self.client.send_goal(goal)
        state = self.client.get_state()
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)
            state = self.client.get_state()
        return self.client.get_result()

    #Funcion para desplazar el robot a la posicion x,y
    def moveTo(self, x, y):
        #mensaje dle tipo MoveBaseGoal y su definicion
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x   
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        #Suscriptor del topic comando para detener la accion
        rospy.Subscriber("comando", String, self.escucharComando)

        self.client.send_goal(goal)
        #En lugar de quedarnos bloqueados esperando que termine el goal
        #nos metemos en un bucle que nos da la oportunidad de escuchar mensajes
        state = self.client.get_state()
        print("moveTo")
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)
            state = self.client.get_state()
        return self.client.get_result()

    #Funcion para buscar el codigo QR y esperar la respuesta del topic qrcode/info
    def QRcode(self):
        print("QRcode start")
        #Mensaje de tipo Twist, su definicion y su publicacion
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 1
        self.pub_velocity.publish(cmd) #Se publica el mensaje
        
        #Mensaje para iniciar la publicacion de informacion de la camara
        self.startcamera.publish("start")
        
        start_time = time.time()
        current_time = time.time()
        elapsed_time = 0
        
        #5 segundos de espera para completar una vuelta completa sobre si mismo
        while elapsed_time<5:
            self.pub_velocity.publish(cmd) #Se publica el mensaje
            current_time = time.time()
            elapsed_time = current_time - start_time
        
        #Detener la publicacion de informacion de la camara
        self.startcamera.publish("stop")
        #Detener el movimiento del robot sobre si mismo
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.pub_velocity.publish(cmd) #Se publica el mensaje

        #Almacenar la informacion correspondiente en el txt
        package_code = open("packages.txt","a")
        package_code.write("none\n")
        package_code.close()
        self.qrcode = False
        print("QRcode end")

    #Callback para cuando se encuentra un QR
    def qrfound(self,msg):
        #Transformar el msg en un string que solo guarde la informacion del QR
        data = str(msg)
        data_ = data[7:-1]
        
        print("QRfound")
        #Detener el giro del robot sobre si mismo
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.pub_velocity.publish(cmd) #Se publica el mensaje
        #Detener la publicacion de informacion de la camara
        self.startcamera.publish("stop")
        #Almacenar la informacion correspondiente en el txt
        package_code = open("packages.txt","a")
        package_code.write(data_+"\n")
        package_code.close()
        #Variable que almacena codigo QR encontrado
        self.qrcode = True

if __name__ == "__main__":
    #Puntos a explorar donde comprobar la existencia de codigo QR , se pueden añadir tantos como se quiera
    points = [(4.06,-2.78),(1.64,1.4)]
    #Iniciar nodo
    rospy.init_node('clientemovebase')
    #variable de la clase ClienteMoveBase
    cliente = ClienteMoveBase()
    # Se dirige a la posicion home o inicial
    cliente.home()
    #Vector que almacena el historico de existencia o no de paquete para todos los puntos
    register_points = []
    f = open("register.txt")
    code_QR = ""
    
    #Para cada punto
    for point in points:
        #Alcanzar la posicion deseada
        result = cliente.moveTo(float(point[0]), float(point[1]))
        #Leer del fichero register.txt el historico de esa posicion
        line = f.readline()
        #Buscar QR en esa poscion
        cliente.QRcode()
        #Actualizar el historico de esa posicion
        if cliente.qrcode:
            line = line[0:-1] + "1 "
        else:
            line = line[0:-1] + "0 "
        #Añadir el historico actualizado de esa posicion al vector de historicos
        register_points.append(line)
        print("result ",result)
    f.close()
    print(register_points)
    #Actualizar el ficher register.txt
    f = open("register.txt","w")
    for line in register_points:
        f.write(line + "\n")
    f.close()
            
    print("DONE")
    
