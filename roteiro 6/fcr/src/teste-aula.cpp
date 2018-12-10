#include "ros/ros.h"
#include <string>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

std_msgs::Float32 vright, vleft;

void rightVelocityCallback(const std_msgs::Float32::ConstPtr& msg)
{
    vright.data = msg->data;
}

void leftVelocityCallback(const std_msgs::Float32::ConstPtr& msg)
{
    vleft.data = msg->data;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "forward_kinematic_pioneer");
    ros::NodeHandle n;
    std::string right = "/v_right";
    std::string left = "/v_left";
    ros::Publisher vel_right = n.advertise<std_msgs::Float32>("/v_right", 1000);
    ros::Publisher vel_left = n.advertise<std_msgs::Float32>("/v_left", 1000);
    //ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Rate loop_rate(10000);
    
    int x = 0;
    float vel_lin, vel_ang;
    std_msgs::Float32 velocidade_direita;
    std_msgs::Float32 velocidade_esquerda;

    loop_rate.sleep();
    ros::spinOnce();
    //vright.data = vleft.data = 10.0;
    //vel_lin = vel_ang = 5.0;

    while(ros::ok())
    {
        // forward kinematics
        //vel_lin = vright.data/2 + vleft.data/2;
        //vel_ang = (vright.data - vleft.data)/0.4;
	velocidade_direita.data = 0.0;
        velocidade_esquerda.data = 0.0;
        //geometry_msgs::Twist vel;
        //vel.linear.x = vel_lin;
        //vel.angular.z = vel_ang;
	if(x<10){
            velocidade_direita.data = 1.0;
            velocidade_esquerda.data = 1.0;
	}
	vel_right.publish(velocidade_direita);
	vel_left.publish(velocidade_esquerda);
        //vel_pub.publish(vel);
	x++;
        loop_rate.sleep();
        ros::spinOnce();
    }
}
