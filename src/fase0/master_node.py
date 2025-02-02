#!/usr/bin/python3
import math
import os
import time

import numpy as np
import rosnode
import rospy  # biblioteca padrao para trabalhar com ros em python
# from custom_msgs.msg import BoundingBox, BoundingBoxes
from clover.srv import GetTelemetry, GetTelemetryRequest
from custom_msgs.srv import (Map2DPoint, Map2DPointRequest, MapEdit,
                            MapEditRequest)
# from std_msgs.msg import Bool
# from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose2D
from mavros_msgs.srv import CommandBool, CommandBoolRequest
# from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, SetBoolRequest
from tools_node import Tools
import colorful as cf
from states import *

class Master(Tools):
    def _init_(self) -> None:
        super().__init__()

    def update(self):

        if self.fsm == 'TakeOff':
            self.current_state = 'TakeOff'
            if (self.start_phase):
                print(cf.blue("ESTADO - TakeOff"))
                print(cf.yellow("Iniciando TakeOff"))
                self.navigateWait(z=1.5, frame_id='body', speed = 0.2, auto_arm=True)
                print(cf.yellow("TakeOff Complete"))

                #Parte responsavel pela troca de estado na fsm
                self.current_state = ''
                self.fsm.add('TakeOff_complete')

        if self.fsm == 'Foward':
            self.current_state = 'Foward'
            print(cf.blue("ESTADO - Foward"))
            print(cf.yellow("Indo 2 metros para frente"))
            self.navigateWait(x=2.0, frame_id='body', speed = 0.2, auto_arm=True)
            print(cf.yellow("Foward Complete"))         
            #Parte responsavel pela troca de estado na fsm
            self.current_state = ''
            self.fsm.add('Foward_complete')

        if self.fsm == 'Land':
            self.land()
            print(cf.blue("ACABOU - LAND FINAL"))
            self.current_state = ''
            self.fsm.add('finished')
           

        if self.fsm != self.current_state:
            self.fsm.updateEvent() 



def main(): 
    rospy.init_node('masterNode', anonymous=True)
    mestre = Master()
    mestre.setSubscribers()
    mestre.setPublishers()
    mestre.setClients()
    mestre.setServer()
    rospy.sleep(10)
    print(cf.yellow("Aguardando chamada do serviço /start_phase"))
    while not rospy.is_shutdown():
        mestre.update()
        if (self.fsm == 'Finish'):
            break


if __name__ == '__main__':
    main()