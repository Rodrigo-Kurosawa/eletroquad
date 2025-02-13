#!/usr/bin/python3
import math
import os
import time
import numpy as np
import rosnode
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from tf.transformations import quaternion_from_euler
from states import TakeOff
from std_msgs.msg import Int32
from clover import srv
from std_srvs.srv import Trigger, SetBool, SetBoolRequest
import colorful as cf

orbit_radius = 2  # Raio da órbita (metros)
orbit_speed = 0.1  # Velocidade de rotação (radianos por segundo)
altitude = 2
current_angle = 0  # Ângulo inicial da órbita

class Tools():
    def __init__(self) -> None:
        self.fsm = TakeOff()
        self.start_phase = False
        self.center_x = 2  # Ponto central de órbita X
        self.center_y = 0
        self.giro_x = 0
        self.erro = 10
        self.there_is_a_bar = False
        self.cores = ["vermelho", "azul", "rosa", "preto"]

    def setSubscribers(self):
        #verifica o quanto se deve girar para centralizar a barra
        self.bar_pos = rospy.Subscriber('/bar_pos', Float32, self.barPose)
        #verifica se a barra está na tela
        self.bar = rospy.Subscriber('/is_on_stream', Bool, self.findBar)
    def setPublishers(self):
        self.pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.cor = rospy.Publisher('/target_color', String, queue_size=10)
    def setClients(self):
        self.navigate = rospy.ServiceProxy('/navigate', srv.Navigate, persistent=True)
        self.land = rospy.ServiceProxy('/land', Trigger, persistent=True)
        self.get_telemetry = rospy.ServiceProxy('/get_telemetry', srv.GetTelemetry, persistent=True)       
        
    def setServer(self):
        rospy.Service('/start_phase', CommandBool, self.startPhase)
        
    def startPhase(self, req):
        self.start_phase = True
        return (True,0)
    
    def findBar(self,bool):
        self.there_is_a_bar = bool

    def barPose(self,data):
        if self.there_is_a_bar == True:
            self.giro_x = data.x
        else:
            #valor inválido
            self.giro_x = 0
        
    def navigateWait(self, x=0, y=0, z=0, yaw=0, speed=0.2, frame_id='map', tolerance=0.1, auto_arm=True, z_participation = 1):
        '''Navigate without interruption, wait until target is reached '''
        try:
            res = self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
            if not res.success:
                rospy.logerr("Falha na navegação")
                return res
            rospy.loginfo("Navegação iniciada")
            while not rospy.is_shutdown():
                telem = self.get_telemetry(frame_id='navigate_target')
                distance = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)
                # rospy.loginfo(f"Distância até o alvo: {distance:.2f} m")
                if distance < tolerance:
                    rospy.loginfo("Alvo alcançado")
                    return res
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao chamar o serviço de navegação: {e}")

    def update_orbit(self, pose, start_time):
        t = time.time() - start_time
        theta = orbit_speed * t  # Ângulo atual

        # Calcula a nova posição (movimento circular)
        pose.pose.position.x = self.center_x + orbit_radius * math.cos(theta)
        pose.pose.position.y = self.center_y + orbit_radius * math.sin(theta)
        pose.pose.position.z = altitude

        # Calcula o yaw para sempre olhar para o centro
        yaw_angle = theta + math.pi  # Inverte a direção para olhar para dentro
        quat = quaternion_from_euler(0, 0, yaw_angle)

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        # Publica a nova posição
        pose.header.stamp = rospy.Time.now()
        return pose
    
    #permite que o drone rotacione a uma velocidade constante(até encontrar a barra) consertar
    def keep_yaw(self, yaw_rate, speed =0.2 ,frame_id = 'map', auto_arm = True):
        try:
            res = self.navigate(yaw=yaw_rate, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
            if not res.success:
                rospy.logerr("Falha na navegação")
                return res
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao chamar o serviço de navegação: {e}")