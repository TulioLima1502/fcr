TRABALHO 1
========
Movimentação de robô dentro de uma ambiente simulado(stage), de forma que o mesmo tenha como desviar de obstáculos(paredes e objetos pontuais) que aparecerem na trajetória até o objetivo.

Entrada
-------
Coordenadas para qual o robô deve se movimentar, em x, y, e o ângulo final(requisitadas do usuário) e mensagem (nav_msgs.msg/Odometry) no tópico /pose.
Além das mensagens do sensor Lidar (sensor_msgs.msg.LaserScan) no tópico /hokuyo_scan.

Saida
-----
Velocidade de comando para o Pioneer no tópico /cmd_vel de tipo geometry_msgs/Twist.

Dependencias do ROS
-------------------
* rospy
* geometry_msgs.msg
* nav_msgs.msg
* tf
* geometry_msgs.msg(Twist)


Dependencias fora do ROS
------------------------
* time import sleep
* tf.transformations import euler_from_quaternion
* math

Algoritmo
---------
O algoritmo implementado não foi retirado de nenhuma referência, foi criado e desenvolvido pelo autor para o trabalho 1 de FCR de 2018/2


Início
    posta_posição_atual
    recebe_dados_usuário
    orienta()
    movimenta()

    movimenta()
    orientacao_final()

Fim

    orienta():
      alinha_destino
    orientacao_final():
      alinha_angulo_final
    movimenta():
      enquanto achou_objeto == False:
        move_na_direção
      fim-enquanto
      se objeto_direita:
	move_esquerda
      senão se objeto_esquerda:
        move_direita
      senão:
        move_direita
      fim-se
      orienta()
      movimenta()

Descrição dos arquivos
----------------------

launch/
    |--> stage-pioneer-3at-hokuyo.launch: launch file que abre todo o ambiente
src/
    |--> 120054337_localizacao.py: source com toda a lógica do programa


trabalho1.avi: Video com um exemplo de funcionamento do robô
trabalho1.pdf: Relatório com descriçao do problema e detalhamento da solução

CMakeLists.txt: Arquivo de configuração da build deste pacote
package.xml: Arquivo de configuração de dependecias deste pacote e informações de versão, autor e descrição
README.md: Este arquivo
