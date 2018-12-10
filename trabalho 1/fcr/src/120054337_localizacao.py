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

    def Position(self, odom_data):
        # rospy.sleep(1)
        self.curr_time = odom_data.header.stamp
        self.pose = odom_data.pose.pose  # the x,y,z pose and quaternion orientation

    def Laser(self, odom_data):
        self.curr_time_laser = odom_data.header.stamp
        self.ranges = odom_data.ranges  # the x,y,z pose and quaternion orientation
        # print(self.ranges)

    def Move_direita(self):
        # pega o angulo atual e subtrai em 45
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
        # print(math.degrees(yaw))
        angulo_desejado = math.degrees(yaw) + 90
        #print(angulo_atual)


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


        vel_movimentacao.linear.x = 0.2
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
                #print(posicao_start_x - self.pose.position.x , posicao_start_y - self.pose.position.y)
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
        # Encontrar objetos no range com menos de 1 metro
        # Virar para direita ou esquerda
        # Ate chegar no ponto desejado

    def Orientando(self,anguloatual):
        x = self.destino_x - self.pose.position.x
        y = self.destino_y - self.pose.position.y
        angulo_aux = y/x
        angulo_orientacao = math.degrees(math.atan(angulo_aux))
        angulo_desejado = angulo_orientacao

        if ( x > 0 ) and ( y > 0 ):
            #print( angulo_orientacao )
            angulo_desejado = angulo_orientacao
        elif ( x < 0 ) and ( y > 0 ):
            #print( 180 + angulo_orientacao )
            angulo_desejado = 180 + angulo_orientacao
        elif ( x < 0 ) and ( y < 0 ):
            #print( - 180 + angulo_orientacao )
            angulo_desejado = - 180 + angulo_orientacao
        elif ( x > 0 ) and ( y < 0 ):
            #print(angulo_orientacao)
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
            # print(math.degrees(yaw))
            angulo_atual= math.degrees(yaw)

            if( angulo_desejado > 0 ):
                vel_orientacao.angular.z = 0.1
            elif( angulo_desejado < 0 ):
                vel_orientacao.angular.z = -0.1

            velocity_orientacao.publish(vel_orientacao)
            
            #print(abs(angulo_atual - angulo_desejado))

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
            # print(math.degrees(yaw))
            angulo_atual= math.degrees(yaw)

            if( angulo_desejado > 0 ):
                vel_orientacao.angular.z = 0.1
            elif( angulo_desejado < 0 ):
                vel_orientacao.angular.z = -0.1

            velocity_orientacao.publish(vel_orientacao)
            
            #print(abs(angulo_atual - angulo_desejado))

            if ( abs(angulo_atual - angulo_desejado) < 0.9 ):
                align = False

        if (abs(angulo_atual - angulo_final) < 0.9):
            print("Alinhado")

    def move(self):

        print("Iniciando o Programa para mover o Pioneer")
        
        rospy.init_node('robot_cleaner', anonymous=True)
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        odoSub = rospy.Subscriber(
            'pose', nav_msgs.msg.Odometry, self.Position, queue_size=1)
        odoLaser = rospy.Subscriber(
            'hokuyo_scan', sensor_msgs.msg.LaserScan, self.Laser, queue_size=1)
        vel_msg = Twist()

        sleep(2)
        print("Posicao: ")

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        teta = math.degrees(yaw)
        print("X: " + str(self.pose.position.x), "Y: " + str(self.pose.position.y), "Graus: " + str(teta))

        posicaoFinalX = input("Qual a posicao final em x desejada?")
        posicaoFinalY = input("Qual a posicao final em y desejada?")
        posicaoFinalTeta = input("Qual o angulo(Graus) final desejado?")
        distanciax = posicaoFinalX - self.pose.position.x
        distanciay = posicaoFinalY - self.pose.position.y
        angulo = posicaoFinalTeta - teta
        print(distanciax,distanciay,angulo)

        self.destino_x = posicaoFinalX
        self.destino_y = posicaoFinalY

        self.Orientando(teta)
        self.Movimentos(teta)

        distanciax = posicaoFinalX - self.pose.position.x
        distanciay = posicaoFinalY - self.pose.position.y

        self.Movimentos(teta)
        self.Orientacao_final(distanciax,distanciay,posicaoFinalTeta)

        exit()

if __name__ == '__main__':
    try:
        myRobot = pioneer()
        myRobot.move()
    except rospy.ROSInterruptException:
        pass
