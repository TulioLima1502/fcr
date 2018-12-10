#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import nav_msgs.msg
import tf
from time import sleep
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

import sensor_msgs.msg

class pioneer():
    def __init__(self, debug=False ,ser= None):
        self.pose = None
        self.curr_time = None
        self.curr_time_laser = None
        self.ranges = None
        self.destino_x = None
        self.destino_y = None
        self.areas = []
        self.graph_dependencias = {}
        self.graph_posicoes = {}
        self.no_atual = None
        self.nos_adjacentes = None
        self.coordenadas_destino = None
        self.destino_final = None
        self.rota_pontos = None
        self.posicaoFinalTeta = None

    def Position(self, odom_data):
        # rospy.sleep(1)
        self.curr_time = odom_data.header.stamp
        self.pose = odom_data.pose.pose  # the x,y,z pose and quaternion orientation

    def Laser(self, odom_data):
        self.curr_time_laser = odom_data.header.stamp
        self.ranges = odom_data.ranges  # the x,y,z pose and quaternion orientation

    def Move_direita(self):
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        angulo_desejado = math.degrees(yaw) - 90

        velocity_direita = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel_direita = Twist()

        vel_direita.linear.x = 0
        vel_direita.linear.y = 0
        vel_direita.linear.z = 0
        vel_direita.angular.x = 0
        vel_direita.angular.y = 0
        vel_direita.angular.z = 0.1

        align = True

        while(align == True):
            (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
            angulo_atual = math.degrees(yaw)
            if( angulo_desejado > 0 ):
                vel_direita.angular.z = 0.1
            elif( angulo_desejado < 0 ):
                vel_direita.angular.z = -0.1
            velocity_direita.publish(vel_direita)
            if ( abs(angulo_atual - angulo_desejado) < 0.9 ):
                align = False
        
        vel_direita.linear.x = 0.2
        vel_direita.angular.z = 0

        distancia = 0
        posicao_atual_x = self.pose.position.x
        posicao_atual_y = self.pose.position.y

        while distancia < 0.2 :
            velocity_direita.publish(vel_direita)
            distancia = math.sqrt( math.pow(( self.pose.position.x - posicao_atual_x ) , 2 ) + math.pow(( self.pose.position.y - posicao_atual_y ) , 2 ) )
        vel_direita.linear.x = 0
        velocity_direita.publish(vel_direita)

    def Move_esquerda(self):
        # pega o angulo atual e soma em 90
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        angulo_desejado = math.degrees(yaw) + 90

        velocity_esquerda = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel_esquerda = Twist()

        vel_esquerda.linear.x = 0
        vel_esquerda.linear.y = 0
        vel_esquerda.linear.z = 0
        vel_esquerda.angular.x = 0
        vel_esquerda.angular.y = 0
        vel_esquerda.angular.z = 0.1

        align = True

        while(align == True):
            (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
            angulo_atual = math.degrees(yaw)
            if( angulo_desejado > 0 ):
                vel_esquerda.angular.z = 0.1
            elif( angulo_desejado < 0 ):
                vel_esquerda.angular.z = -0.1
            velocity_esquerda.publish(vel_esquerda)
            if ( abs(angulo_atual - angulo_desejado) < 0.9 ):
                align = False

        vel_esquerda.linear.x = 0.2
        vel_esquerda.angular.z = 0

        distancia = 0
        posicao_atual_x = self.pose.position.x
        posicao_atual_y = self.pose.position.y

        while distancia < 0.2 :
            velocity_esquerda.publish(vel_esquerda)
            distancia = math.sqrt( math.pow(( self.pose.position.x - posicao_atual_x ) , 2 ) + math.pow(( self.pose.position.y - posicao_atual_y ) , 2 ) )
        vel_esquerda.linear.x = 0
        velocity_esquerda.publish(vel_esquerda)

    def Movimentos(self,anguloatual):
        x = self.destino_x - self.pose.position.x
        y = self.destino_y - self.pose.position.y
        distancia_total = math.sqrt( y * y + x * x )

        velocity_movimentacao = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel_movimentacao = Twist()


        vel_movimentacao.linear.x = 0.3
        vel_movimentacao.linear.y = 0
        vel_movimentacao.linear.z = 0
        vel_movimentacao.angular.x = 0
        vel_movimentacao.angular.y = 0
        vel_movimentacao.angular.z = 0

        posicao_atual_x = self.pose.position.x
        posicao_atual_y = self.pose.position.y
        distancia_atual_percorrida = 0.0

        moving = True
        while moving:
            velocity_movimentacao.publish(vel_movimentacao)
            distancia_atual_percorrida = math.sqrt( math.pow(( self.pose.position.x - posicao_atual_x ) , 2 ) + math.pow(( self.pose.position.y - posicao_atual_y ) , 2 ) )

            moving = self.Reorientar(posicao_atual_x,posicao_atual_y)

            if (distancia_total - distancia_atual_percorrida)<0:
                moving = False
            
        vel_movimentacao.linear.x = 0
        velocity_movimentacao.publish(vel_movimentacao)

    def Reorientar(self,posicao_start_x,posicao_start_y):
        lista_range = self.ranges
        for item in lista_range:
            if(float(item) - 0.5 < 0):
                print(item)
                if (lista_range[75] > lista_range[185]): 
                    self.Move_direita()
                elif (lista_range[75] < lista_range[185]): 
                    self.Move_esquerda()
                else:
                    self.Move_direita()

                distanciax = self.destino_x - self.pose.position.x
                distanciay = self.destino_y - self.pose.position.y
                (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
                angulo_atual= math.degrees(yaw)
                self.Orientando(angulo_atual)
                self.Movimentos(angulo_atual)
                return False
        return True

    def Orientando(self,anguloatual):
        x = self.destino_x - self.pose.position.x
        y = self.destino_y - self.pose.position.y
        angulo_aux = y/x
        angulo_orientacao = math.degrees(math.atan(angulo_aux))
        angulo_desejado = angulo_orientacao

        if ( x > 0 ) and ( y > 0 ):
            angulo_desejado = angulo_orientacao
        elif ( x < 0 ) and ( y > 0 ):
            angulo_desejado = 180 + angulo_orientacao
        elif ( x < 0 ) and ( y < 0 ):
            angulo_desejado = - 180 + angulo_orientacao
        elif ( x > 0 ) and ( y < 0 ):
            angulo_desejado = angulo_orientacao

        velocity_orientacao = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel_orientacao = Twist()

        vel_orientacao.linear.x = 0
        vel_orientacao.linear.y = 0
        vel_orientacao.linear.z = 0
        vel_orientacao.angular.x = 0
        vel_orientacao.angular.y = 0
        vel_orientacao.angular.z = 0.1

        align = True

        while(align == True):
            (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
            angulo_atual= math.degrees(yaw)

            if( angulo_desejado > 0 ):
                vel_orientacao.angular.z = 0.1
            elif( angulo_desejado < 0 ):
                vel_orientacao.angular.z = -0.1

            velocity_orientacao.publish(vel_orientacao)

            if ( abs(angulo_atual - angulo_desejado) < 0.9 ):
                align = False
        vel_orientacao.angular.z = 0
        velocity_orientacao.publish(vel_orientacao)

    def Orientacao_final(self,x,y,angulo_final):
        angulo_aux = y/x
        angulo_desejado = angulo_final

        velocity_orientacao = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel_orientacao = Twist()

        vel_orientacao.linear.x = 0
        vel_orientacao.linear.y = 0
        vel_orientacao.linear.z = 0
        vel_orientacao.angular.x = 0
        vel_orientacao.angular.y = 0
        vel_orientacao.angular.z = 0.1

        align = True

        while(align == True):
            (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
            angulo_atual= math.degrees(yaw)

            if( angulo_desejado > 0 ):
                vel_orientacao.angular.z = 0.1
            elif( angulo_desejado < 0 ):
                vel_orientacao.angular.z = -0.1

            velocity_orientacao.publish(vel_orientacao)

            if ( abs(angulo_atual - angulo_desejado) < 0.9 ):
                align = False

        if (abs(angulo_atual - angulo_final) < 0.9):
            print("Alinhado")

    def Grafo(self):
        f = open("grafo",'r')
        texto = f.readlines()
        x = 0
        while x < len(texto):
            if texto[x] == "\n":
                local = texto.index(texto[x])
                texto.pop(local)
            else:
                texto[x] = texto[x].split('\t')
                x += 1
        
        for i in texto:
            local = texto.index(i) # Local do i em texto
            for b in i:
                local2 = texto[local].index(b) # Local2 do b em i ( local )
                if "\n" in b:
                    texto[local][local2] = b.replace("\n",'') # Substitui o valor de acordo com "local" e "local2"
        i = len(texto)
        aux = 0
        while(aux < i):
            coordenadas_1_x, coordenadas_1_y = texto[aux][1].split(',')
            coordenadas_2_x, coordenadas_2_y = texto[aux][2].split(',')
            coordenadas_3_x, coordenadas_3_y = texto[aux][3].split(',')
            coordenadas_4_x, coordenadas_4_y = texto[aux][4].split(',')
            media_superior_x = (float(coordenadas_1_x) + float(coordenadas_2_x))/2
            media_superior_y = (float(coordenadas_1_y) + float(coordenadas_2_y))/2
            media_inferior_x = (float(coordenadas_3_x) + float(coordenadas_4_x))/2
            media_inferior_y = (float(coordenadas_3_y) + float(coordenadas_4_y))/2
            ponto_central_no_x = (media_inferior_x + media_superior_x)/2
            ponto_central_no_y = (media_inferior_y + media_superior_y)/2
            texto[aux].append("central: {0},{1}".format(ponto_central_no_x,ponto_central_no_y))
            aux=aux+1

        aux = 0
        grafo = []

        grafo = texto[:]
        while(aux < i):
            grafo[aux].pop(1)
            grafo[aux].pop(1)
            grafo[aux].pop(1)
            grafo[aux].pop(1)
            self.graph_dependencias[str(grafo[aux][0])] = str(grafo[aux][1].split(','))
            self.graph_posicoes[str(grafo[aux][0])] = grafo[aux][2]
            aux = aux + 1

        #print(self.graph_dependencias.keys())

    def OndeEstou(self):
        f = open("grafo",'r')
        texto = f.readlines()
        x = 0
        while x < len(texto):
            if texto[x] == "\n":
                local = texto.index(texto[x])
                texto.pop(local)
            else:
                texto[x] = texto[x].split('\t')
                x += 1
        
        for i in texto:
            local = texto.index(i) # Local do i em texto
            for b in i:
                local2 = texto[local].index(b) # Local2 do b em i ( local )
                if "\n" in b:
                    texto[local][local2] = b.replace("\n",'') # Substitui o valor de acordo com "local" e "local2"
        i = len(texto)
        aux = 0
        while(aux < i):
            coordenadas_1_x, coordenadas_1_y = texto[aux][1].split(',')
            coordenadas_2_x, coordenadas_2_y = texto[aux][2].split(',')
            coordenadas_3_x, coordenadas_3_y = texto[aux][3].split(',')
            coordenadas_4_x, coordenadas_4_y = texto[aux][4].split(',')
            media_superior_x = (float(coordenadas_1_x) + float(coordenadas_2_x))/2
            media_superior_y = (float(coordenadas_1_y) + float(coordenadas_2_y))/2
            media_inferior_x = (float(coordenadas_3_x) + float(coordenadas_4_x))/2
            media_inferior_y = (float(coordenadas_3_y) + float(coordenadas_4_y))/2
            ponto_central_no_x = (media_inferior_x + media_superior_x)/2
            ponto_central_no_y = (media_inferior_y + media_superior_y)/2
            texto[aux].append("central: {0},{1}".format(ponto_central_no_x,ponto_central_no_y))
            aux = aux + 1
        aux = 0
        while(aux < i):
            coordenadas_1_x, coordenadas_1_y = texto[aux][1].split(',')
            coordenadas_2_x, coordenadas_2_y = texto[aux][2].split(',')
            coordenadas_3_x, coordenadas_3_y = texto[aux][3].split(',')
            coordenadas_4_x, coordenadas_4_y = texto[aux][4].split(',')

            y = (float(coordenadas_2_y) - float(coordenadas_1_y))
            x = (float(coordenadas_2_x) - float(coordenadas_1_x))

            angulo_aux = y/x
            angulo_orientacao1_2 = math.degrees(math.atan(angulo_aux))
            
            if ( x > 0 ) and ( y > 0 ):
                angulo_orientacao1_2 = angulo_orientacao1_2
            elif ( x < 0 ) and ( y > 0 ):
                angulo_orientacao1_2 = 180 + angulo_orientacao1_2
            elif ( x < 0 ) and ( y < 0 ):
                angulo_orientacao1_2 = 180 - angulo_orientacao1_2
            elif ( x > 0 ) and ( y < 0 ):
                angulo_orientacao1_2 = 360 + angulo_orientacao1_2

            y = float(self.pose.position.y) - float(coordenadas_1_y) 
            x = float(self.pose.position.x) - float(coordenadas_1_x)
            angulo_aux = y/x
            angulo_orientacao = math.degrees(math.atan(angulo_aux))

            if ( x > 0 ) and ( y > 0 ):
                angulo_orientacao = angulo_orientacao
            elif ( x < 0 ) and ( y > 0 ):
                angulo_orientacao = 180 + angulo_orientacao
            elif ( x < 0 ) and ( y < 0 ):
                angulo_orientacao = 180 - angulo_orientacao
            elif ( x > 0 ) and ( y < 0 ):
                angulo_orientacao = 360 + angulo_orientacao

            if(angulo_orientacao - angulo_orientacao1_2 > 0):
                y = (float(coordenadas_4_y) - float(coordenadas_1_y)) 
                x = (float(coordenadas_4_x) - float(coordenadas_1_x))  
                angulo_aux = y/x
                angulo_orientacao1_4 = math.degrees(math.atan(angulo_aux))
                
                if ( x > 0 ) and ( y > 0 ):
                    angulo_orientacao1_4 = angulo_orientacao1_4
                elif ( x < 0 ) and ( y > 0 ):
                    angulo_orientacao1_4 = 180 + angulo_orientacao1_4
                elif ( x < 0 ) and ( y < 0 ):
                    angulo_orientacao1_4 = 180 - angulo_orientacao1_4
                elif ( x > 0 ) and ( y < 0 ):
                    angulo_orientacao1_4 = 360 + angulo_orientacao1_4

                if( angulo_orientacao - angulo_orientacao1_4 > 0 ):

                    y = (float(coordenadas_2_y) - float(coordenadas_3_y)) 
                    x = (float(coordenadas_2_x) - float(coordenadas_3_x))  
                    
                    angulo_aux = y/x
                    angulo_orientacao3_2 = math.degrees(math.atan(angulo_aux))

                    if ( x > 0 ) and ( y > 0 ):
                        angulo_orientacao3_2 = angulo_orientacao3_2
                    elif ( x < 0 ) and ( y > 0 ):
                        angulo_orientacao3_2 = 180 + angulo_orientacao3_2
                    elif ( x < 0 ) and ( y < 0 ):
                        angulo_orientacao3_2 = 180 - angulo_orientacao3_2
                    elif ( x > 0 ) and ( y < 0 ):
                        angulo_orientacao3_2 = 360 + angulo_orientacao3_2
                    
                    y = float(self.pose.position.y) - float(coordenadas_3_y) 
                    x = float(self.pose.position.x) - float(coordenadas_3_x)
                    angulo_aux = y/x
                    angulo_orientacao = math.degrees(math.atan(angulo_aux))

                    if ( x > 0 ) and ( y > 0 ):
                        angulo_orientacao = angulo_orientacao
                    elif ( x < 0 ) and ( y > 0 ):
                        angulo_orientacao = 180 + angulo_orientacao
                    elif ( x < 0 ) and ( y < 0 ):
                        angulo_orientacao = 180 - angulo_orientacao
                    elif ( x > 0 ) and ( y < 0 ):
                        angulo_orientacao = 360 + angulo_orientacao

                    #print(angulo_orientacao)

                    y = (float(coordenadas_4_y) - float(coordenadas_3_y)) 
                    x = (float(coordenadas_4_x) - float(coordenadas_3_x))  
                    
                    angulo_aux = y/x
                    angulo_orientacao3_4 = math.degrees(math.atan(angulo_aux))

                    #print(angulo_orientacao3_4)

                    if ( x > 0 ) and ( y > 0 ):
                        angulo_orientacao3_4 = angulo_orientacao3_4
                    elif ( x < 0 ) and ( y > 0 ):
                        angulo_orientacao3_4 = 180 + angulo_orientacao3_4
                    elif ( x < 0 ) and ( y < 0 ):
                        angulo_orientacao3_4 = 180 + angulo_orientacao3_4
                    elif ( x > 0 ) and ( y < 0 ):
                        angulo_orientacao3_4 = 360 + angulo_orientacao3_4

                    #print(angulo_orientacao3_4)

                    if(angulo_orientacao3_4 - angulo_orientacao > 0) and (angulo_orientacao3_2 - angulo_orientacao < 0):
                        print("\nO Robo esta no noh: {}".format(str(texto[aux][0])))
                        self.no_atual = texto[aux][0]

            elif(angulo_orientacao - angulo_orientacao1_2 <= 0):
                
                y = (float(coordenadas_2_y) - float(coordenadas_3_y)) 
                x = (float(coordenadas_2_x) - float(coordenadas_3_x))  
                    
                angulo_aux = y/x
                angulo_orientacao3_2 = math.degrees(math.atan(angulo_aux))

                if ( x > 0 ) and ( y > 0 ):
                    angulo_orientacao3_2 = angulo_orientacao3_2
                elif ( x < 0 ) and ( y > 0 ):
                    angulo_orientacao3_2 = 180 + angulo_orientacao3_2
                elif ( x < 0 ) and ( y < 0 ):
                    angulo_orientacao3_2 = 180 - angulo_orientacao3_2
                elif ( x > 0 ) and ( y < 0 ):
                    angulo_orientacao3_2 = 360 + angulo_orientacao3_2

                y = float(self.pose.position.y) - float(coordenadas_3_y) 
                x = float(self.pose.position.x) - float(coordenadas_3_x)
                angulo_aux = y/x
                angulo_orientacao = math.degrees(math.atan(angulo_aux))

                if ( x > 0 ) and ( y > 0 ):
                    angulo_orientacao = angulo_orientacao
                elif ( x < 0 ) and ( y > 0 ):
                    angulo_orientacao = 180 + angulo_orientacao
                elif ( x < 0 ) and ( y < 0 ):
                    angulo_orientacao = 180 - angulo_orientacao
                elif ( x > 0 ) and ( y < 0 ):
                    angulo_orientacao = 360 + angulo_orientacao

                y = (float(coordenadas_4_y) - float(coordenadas_3_y)) 
                x = (float(coordenadas_4_x) - float(coordenadas_3_x))  
                    
                angulo_aux = y/x
                angulo_orientacao3_4 = math.degrees(math.atan(angulo_aux))

                if ( x > 0 ) and ( y > 0 ):
                    angulo_orientacao3_4 = angulo_orientacao3_4
                elif ( x < 0 ) and ( y > 0 ):
                    angulo_orientacao3_4 = 180 + angulo_orientacao3_4
                elif ( x < 0 ) and ( y < 0 ):
                    angulo_orientacao3_4 = 180 + angulo_orientacao3_4
                elif ( x > 0 ) and ( y < 0 ):
                    angulo_orientacao3_4 = 360 + angulo_orientacao3_4

                if(angulo_orientacao3_4 > angulo_orientacao) and (angulo_orientacao3_2 < angulo_orientacao):
                    print("\nO Robo esta no noh: {}".format(str(texto[aux][0])))
                    self.no_atual = texto[aux][0]
            aux = aux + 1
            # 1 com 2 o angulo entre 1 e 2 deve ser maior para garantir que esta dentro

        print("Os nos acessiveis do ponto onde voce esta: {}".format(str(self.graph_dependencias.keys())))
            
        return self.no_atual,self.graph_dependencias.keys()
        ## pegar a posicao atual e comparar tentando encaixar em alguma da informacoes de posicoes

    def AndaNos(self,nohs_proximos):
        #print(noh)
        #print(nohs_proximos)
        noh_destino = raw_input("\nPara qual noh voce deseja ir?")
        #print(noh_destino)
        if noh_destino not in self.graph_dependencias.keys():
            print("Nao eh proximo, forneca algum noh adjacente")
            return False

        f = open("grafo",'r')
        texto = f.readlines()
        x = 0
        while x < len(texto):
            if texto[x] == "\n":
                local = texto.index(texto[x])
                texto.pop(local)
            else:
                texto[x] = texto[x].split('\t')
                x += 1
        
        for i in texto:
            local = texto.index(i) # Local do i em texto
            for b in i:
                local2 = texto[local].index(b) # Local2 do b em i ( local )
                if "\n" in b:
                    texto[local][local2] = b.replace("\n",'') # Substitui o valor de acordo com "local" e "local2"
        i = len(texto)
        aux = 0
        while(aux < i):
            coordenadas_1_x, coordenadas_1_y = texto[aux][1].split(',')
            coordenadas_2_x, coordenadas_2_y = texto[aux][2].split(',')
            coordenadas_3_x, coordenadas_3_y = texto[aux][3].split(',')
            coordenadas_4_x, coordenadas_4_y = texto[aux][4].split(',')
            media_superior_x = (float(coordenadas_1_x) + float(coordenadas_2_x))/2
            media_superior_y = (float(coordenadas_1_y) + float(coordenadas_2_y))/2
            media_inferior_x = (float(coordenadas_3_x) + float(coordenadas_4_x))/2
            media_inferior_y = (float(coordenadas_3_y) + float(coordenadas_4_y))/2
            ponto_central_no_x = (media_inferior_x + media_superior_x)/2
            ponto_central_no_y = (media_inferior_y + media_superior_y)/2
            texto[aux].append("central: {0},{1}".format(ponto_central_no_x,ponto_central_no_y))
            aux = aux + 1
            
        achou_no = False
        aux = 0
        while(aux < i):
            if(texto[aux][0] == noh_destino):
                print("O noh de destino estah no ponto: {}".format(texto[aux][6].split('central: ')[1]))
                self.coordenadas_destino = texto[aux][6].split('central: ')[1]
                achou_no= True
                self.destino_final = noh_destino
                self.Rota()
            aux = aux + 1

        if(achou_no == False):
            print("Noh nao valido...  Insira um novo no\n\n")
            self.AndaNos(nohs_proximos)
        
        self.posicaoFinalTeta = input("\nQual o angulo(Graus) final desejado?")

        return True

    def Rota(self):
        grafo = self.graph_dependencias
        melhor_caminho = self.find_shortest_path(grafo,self.no_atual,self.destino_final)
        print("O caminho ate o ponto eh: {}".format(melhor_caminho))
        self.rota_pontos = melhor_caminho

    def find_shortest_path(self, graph, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not graph.has_key(start):
            return None
        shortest = None
        graph_aux = graph[start]
        graph_aux = graph_aux.replace("[","").replace("]","").replace("'","").replace(" ","")
        graph_list = graph_aux.split(",")
        for node in graph_list:
            if node not in path:
                newpath = self.find_shortest_path(graph, node, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest

    def moves_in_nodes(self,noh,teta):
        print("Movendo para o no: {}".format(noh))
        if noh not in self.graph_dependencias.keys():
            print("Nao eh proximo, forneca algum noh adjacente")
            return False

        f = open("grafo",'r')
        texto = f.readlines()
        x = 0
        while x < len(texto):
            if texto[x] == "\n":
                local = texto.index(texto[x])
                texto.pop(local)
            else:
                texto[x] = texto[x].split('\t')
                x += 1
        
        for i in texto:
            local = texto.index(i) # Local do i em texto
            for b in i:
                local2 = texto[local].index(b) # Local2 do b em i ( local )
                if "\n" in b:
                    texto[local][local2] = b.replace("\n",'') # Substitui o valor de acordo com "local" e "local2"
        i = len(texto)
        aux = 0
        while(aux < i):
            coordenadas_1_x, coordenadas_1_y = texto[aux][1].split(',')
            coordenadas_2_x, coordenadas_2_y = texto[aux][2].split(',')
            coordenadas_3_x, coordenadas_3_y = texto[aux][3].split(',')
            coordenadas_4_x, coordenadas_4_y = texto[aux][4].split(',')
            media_superior_x = (float(coordenadas_1_x) + float(coordenadas_2_x))/2
            media_superior_y = (float(coordenadas_1_y) + float(coordenadas_2_y))/2
            media_inferior_x = (float(coordenadas_3_x) + float(coordenadas_4_x))/2
            media_inferior_y = (float(coordenadas_3_y) + float(coordenadas_4_y))/2
            ponto_central_no_x = (media_inferior_x + media_superior_x)/2
            ponto_central_no_y = (media_inferior_y + media_superior_y)/2
            texto[aux].append("central: {0},{1}".format(ponto_central_no_x,ponto_central_no_y))
            aux = aux + 1
            
        achou_no = False
        aux = 0
        while(aux < i):
            if(texto[aux][0] == noh):
                #print("O noh intermediario estah no ponto: {}".format(texto[aux][6].split('central: ')[1]))
                self.coordenadas_destino = texto[aux][6].split('central: ')[1]
                achou_no= True
                self.destino_final = noh
                
                posicaoFinalX = float(self.coordenadas_destino.split(',')[0])
                posicaoFinalY = float(self.coordenadas_destino.split(',')[1])
                
                distanciax = posicaoFinalX - self.pose.position.x
                distanciay = posicaoFinalY - self.pose.position.y

                self.destino_x = posicaoFinalX
                self.destino_y = posicaoFinalY

                self.Orientando(teta)
                self.Movimentos(teta)
            aux = aux + 1

    def move(self):

        print("Iniciando o Programa para mover o Pioneer")
        self.Grafo()
        
        rospy.init_node('robot_cleaner', anonymous=True)
        
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        odoSub = rospy.Subscriber('pose', nav_msgs.msg.Odometry, self.Position, queue_size=1)
        odoLaser = rospy.Subscriber('hokuyo_scan', sensor_msgs.msg.LaserScan, self.Laser, queue_size=1)
        vel_msg = Twist()
        sleep(2)

        #print("Posicao: ")
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        teta = math.degrees(yaw)
        #print("X: " + str(self.pose.position.x), "Y: " + str(self.pose.position.y), "Graus: " + str(teta))
        
        meu_no, nos_proximos = self.OndeEstou()
        encontrou_no = self.AndaNos(nos_proximos)
        while encontrou_no == False:
            encontrou_no = self.AndaNos(nos_proximos)

        noh_destino = self.coordenadas_destino

        for point in self.rota_pontos:
            #print(point)
            self.moves_in_nodes(point,teta)

        posicaoFinalX = float(noh_destino.split(',')[0])
        posicaoFinalY = float(noh_destino.split(',')[1])

        distanciax = posicaoFinalX - self.pose.position.x
        distanciay = posicaoFinalY - self.pose.position.y

        self.Orientacao_final(distanciax,distanciay,self.posicaoFinalTeta)

        exit()

if __name__ == '__main__':
    try:
        myRobot = pioneer()
        myRobot.move()
    except rospy.ROSInterruptException:
        pass
