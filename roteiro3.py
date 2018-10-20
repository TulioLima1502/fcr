#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import nav_msgs.msg
import tf
from time import sleep
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math


class pioneer():
    def __init__(self, debug=False ,ser= None):
        self.pose = None
        self.curr_time = None

    def Position(self, odom_data):
        # rospy.sleep(1)
        self.curr_time = odom_data.header.stamp
        self.pose = odom_data.pose.pose  # the x,y,z pose and quaternion orientation
        # print(curr_time)
        # print(pose)

    def Orientando(self,x,y,anguloatual):
        angulo_aux = y/x
        angulo_orientacao = math.degrees(math.atan(angulo_aux))
        angulo_desejado = angulo_orientacao

        if ( x > 0 ) and ( y > 0 ):
            print( angulo_orientacao)
            angulo_desejado = angulo_orientacao
        elif ( x < 0 ) and ( y > 0 ):
            print( 180 + angulo_orientacao )
            angulo_desejado = 180 + angulo_orientacao
        elif ( x < 0 ) and ( y < 0 ):
            print( - 180 + angulo_orientacao )
            angulo_desejado = - 180 + angulo_orientacao
        elif ( x > 0 ) and ( y < 0 ):
            print(angulo_orientacao)
            angulo_desejado = angulo_orientacao


        velocity_orientacao = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel_orientacao = Twist()

        vel_orientacao.linear.x = 0
        vel_orientacao.linear.y = 0
        vel_orientacao.linear.z = 0
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
            
            print(abs(angulo_atual - angulo_desejado))

            if ( abs(angulo_atual - angulo_desejado) < 0.5 ):
                align = False

    def move(self):

        print("Iniciando o Programa para mover o Pioneer")
        
        rospy.init_node('robot_cleaner', anonymous=True)
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        odoSub = rospy.Subscriber(
            'pose', nav_msgs.msg.Odometry, self.Position, queue_size=1)
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

        self.Orientando(distanciax,distanciay,teta)
        # Fazer 16 testes combinados

        exit()
        vel_msg.linear.x = 0.2 # Setando a velocidade para um valor que nao provoca tantos erros na posicao
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.0

        
        while not rospy.is_shutdown():
            t0 = rospy.Time.now().to_sec()
            current_distance = 1
            final_distance = self.pose.position.x + distanciax
            print("Sua posicao:")
            print(self.pose.position.x, self.pose.position.y, self.pose.orientation.w)
            print("Sua posicao final:")
            print(final_distance)
            sleep(2)

            while (current_distance >= 0.005):
                velocity_publisher.publish(vel_msg)
                # t1 = rospy.Time.now().to_sec()
                # current_distance = speed*(t1-t0)
                current_distance = abs(final_distance - (self.pose.position.x))
                print(current_distance)
                posicaox = self.pose.position.x
                posicaoy = self.pose.position.y
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            # print(posicaox)
            # print(posicaoy)
            exit()
            

if __name__ == '__main__':
    try:
        myRobot = pioneer()
        myRobot.move()
    except rospy.ROSInterruptException:
        pass
