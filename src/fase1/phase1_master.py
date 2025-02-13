#!/usr/bin/python3
import math
import os
import time

import numpy as np
import rosnode
import rospy  # biblioteca padrao para trabalhar com ros em python
# from custom_msgs.msg import BoundingBox, BoundingBoxes
from clover.srv import GetTelemetry, GetTelemetryRequest
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
# from custom_msgs.srv import (Map2DPoint, Map2DPointRequest, MapEdit,
#                             MapEditRequest)
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
                self.navigateWait(z=1.0, frame_id='body', speed = 0.2, auto_arm=True)
                print(cf.yellow("TakeOff Complete"))

                #Parte responsavel pela troca de estado na fsm
                self.current_state = ''
                self.fsm.add('takeOff_complete')
                
        if self.fsm == 'Yaw':
            self.current_state = 'Yaw'
            print(cf.blue("ESTADO - Yaw"))
            print(cf.yellow("Buscando poste..."))
            #verifica se ainda há postes a serem orbitados
            if(self.cores == []):
                print(cf.yellow("All bars passed"))
                self.current_state = ''
                self.fsm.add('no_bars')
            else:
                self.cor.publish(self.cores[0])
                while(self.giro_x > self.erro or not (self.there_is_a_bar)):
                    #print(f"Recebido os seguintes valores{self.giro_x} e {self.there_is_a_bar}")
                    if(self.there_is_a_bar):
                        #gira na direção da barra com redução progressiva da velocidade
                        print("entrou no g")
                        self.keep_yaw(self.giro_x)
                    else:
                        #gira até encontrar uma barra
                        print("entrou no yaw")
                        self.keep_yaw(0.2)
                print(cf.yellow("Cilinder centralized"))  
                #deleta a cor do cilindro que acabou de passar
                del self.cores[0]       
                #Parte responsavel pela troca de estado na fsm
                self.current_state = ''
                self.fsm.add('centralized')
                
        if self.fsm == 'Foward':
            self.current_state = 'Foward'
            print(cf.blue("ESTADO - Foward"))
            print(cf.yellow("Indo para frente"))
            self.navigateWait(x=2.0, y=3.0, frame_id='body', speed = 0.2, auto_arm=True)
            print(cf.yellow("Foward Complete"))  
            # self.center_x = poste_x
            # self.center_y = poste_y
            #Parte responsavel pela troca de estado na fsm
            self.current_state = ''
            self.fsm.add('foward_complete')

        if self.fsm == 'Rotate':
            self.current_state = 'Rotate'
            print(cf.blue("ESTADO - Rotate"))
            print(cf.yellow("Rotacionando em torno do ponto"))
            pose = PoseStamped()
            start_time = time.time()
            while not rospy.is_shutdown():
                self.update_orbit(pose, start_time)
                self.pose_pub.publish(pose)

            print(cf.yellow("Rotate Complete"))        
            #Parte responsavel pela troca de estado na fsm
            self.current_state = ''
            self.fsm.add('rotate_complete')
        
        if self.fsm == 'Land':
            self.land()
            print(cf.blue("Pouso concluído"))
            self.current_state = ''
            self.fsm.add('finished')
           
        if self.fsm == 'Finish':
            print(cf.blue("ESTADO - Finish"))
            rospy.signal_shutdown("Finalizado pelo estado Finish")
        
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

if __name__ == '__main__':
    main()