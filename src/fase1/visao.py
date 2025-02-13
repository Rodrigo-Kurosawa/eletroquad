#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from cv_bridge import CvBridge

class ImageConverter():
    def __init__(self):
        self.bar_pos = rospy.Publisher('/bar_pos', Float32, queue_size=10)
        self.bar = rospy.Publisher('/is_on_stream', Bool, queue_size=10)
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.callback)
        self.cores = rospy.Subscriber('/target_color', String, self.atualiza_cor)
        
        self.cor = None
        self.dicionario_cores = {
            "vermelho": ((0, 0, 100), (100, 10, 255)),
            "azul": ((100, 0, 0), (255, 10, 50)),
            "preto": ((0, 0, 0), (50, 10, 50)),
            "rosa": ((100, 0, 100), (255, 10, 255))
        }
        self.bridge = CvBridge()
        # self.dicionario_cores = {
        #     "vermelho": (0, 0, 255),
        #     "azul": (255, 0, 0),
        #     "preto": (0, 0, 0),
        #     "rosa": (255, 0, 255)
        # }
        
    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if self.cor in self.dicionario_cores:
            cor_inferior, cor_superior = self.dicionario_cores[self.cor]
            #rospy.loginfo(f"Cor alvo recebida: {self.cor} - BGR: {self.cor_alvo}")
        else:
            #rospy.logwarn(f"Cor desconhecida: {cor}")
            return
            
        # Criar a máscara para a cor selecionada
        mask = cv2.inRange(frame, np.array(cor_inferior), np.array(cor_superior))
        
        cv2.imshow('Object Mask', mask)
        cv2.waitKey(1) 
                
        # remove as sujeiras
        kernel = np.ones((5, 5), np.uint8) 
        mask_eroded = cv2.erode(mask, kernel, iterations=3)
        sum_mask = np.sum(mask_eroded)
        print(f"qnt de pixeis {sum_mask}")
        
        #verifica se a barra está na tela(1000 valor arbitrário)
        if sum_mask > 5000:
            self.bar.publish(True)
        else:
            self.bar.publish(False)
            self.bar_pos.publish(0)
            return
        
        # Calcular os momentos da imagem binária
        moments = cv2.moments(mask_eroded)
        cx = 0
        # Verificar se há algum pixel da cor desejada detectado (área > 0)
        if moments["m00"] != 0:
            # Calcular o centro de massa dos pixels 
            cx = int(moments["m10"] / moments["m00"])
            
        # Obtém as dimensões da imagem (altura, largura)
        _, largura, _ = frame.shape
    
        # Calcula o centro da imagem
        centro_x = largura // 2
    
        setpoint = Float32()
        setpoint = cx - centro_x # Define a coordenada x do ponto 
        
        # Publica o setpoint no tópico
        #rospy.loginfo(f"Publicando setpoint: dx={setpoint}")
        self.bar_pos.publish(setpoint)
    
    def atualiza_cor(self, data):
        self.cor = data.data
        print(f"pegou a cor {self.cor}")
        
if __name__ == "__main__":
    rospy.init_node('image_process', anonymous=False)
    rospy.loginfo("Iniciando o node image_process")
    ic = ImageConverter()
    rospy.spin()
    cv2.waitKey()