TRABALHO 2
========
Movimentação de robô dentro de uma ambiente simulado(stage), dentre os diversos nós e constroi as grades de ocupação, assim como também ilustra a posição do usuário dentro das mesmas.

Entrada
-------
Nó para qual o robô deve se movimentar e o ângulo final(requisitadas do usuário) e mensagem (nav_msgs.msg/Odometry) no tópico /pose.
Além das mensagens do sensor Lidar (sensor_msgs.msg.LaserScan) no tópico /hokuyo_scan e do arquivo 'grafo' contendo o mapa topológico.

Saida
-----
Velocidade de comando para o Pioneer no tópico /cmd_vel de tipo geometry_msgs/Twist, imagens ilustrando as grades de ocupação de cada nó.

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
* PIL
* numpy
* threading
* multiprocessing
* os
* gevent
* cv2

Algoritmo
---------
O algoritmo implementado não foi retirado de nenhuma referência, foi criado e desenvolvido pelo autor para o trabalho 2 de FCR de 2018/2

Descrição dos arquivos
----------------------

launch/
    |--> stage-pioneer-3at-hokuyo.launch: launch file que abre todo o ambiente
src/
    |--> trabalho2.py: source com toda a lógica do programa


trabalho2.avi: Video com um exemplo de funcionamento do robô
trabalho2.pdf: Relatório com descriçao do problema e detalhamento da solução

CMakeLists.txt: Arquivo de configuração da build deste pacote
package.xml: Arquivo de configuração de dependecias deste pacote e informações de versão, autor e descrição
README.md: Este arquivo
