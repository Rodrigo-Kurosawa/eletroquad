#!/usr/bin/python3
import math
import os
import time
import numpy as np
import rosnode
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool
from states import TakeOff
from std_msgs.msg import Int32
from clover import srv
from std_srvs.srv import Trigger, SetBool, SetBoolRequest
import colorful as cf

class Tools():
    def __init__(self) -> None:
        self.fsm = TakeOff()
        self.start_phase = False

    def setSubscribers(self):
        None
    def setPublishers(self):
        None
    def setClients(self):
        self.navigate = rospy.ServiceProxy('/navigate', srv.Navigate, persistent=True)
        self.land = rospy.ServiceProxy('/land', Trigger, persistent=True)
        self.get_telemetry = rospy.ServiceProxy('/get_telemetry', srv.GetTelemetry, persistent=True)        
        

    def setServer(self):
        rospy.Service('/start_phase', CommandBool, self.startPhase)

        
    def startPhase(self, req):
        self.start_phase = True
        return (True,0)

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

    