#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <std_msgs/Float32.h>

float position_moteur_1 = 0;
float position_moteur_2 = 0;
float position_moteur_3 = 0;
float position_moteur_4 = 0;
float position_moteur_5 = 0;
float position_moteur_6 = 0;

void Lire_Capteur(const sensor_msgs::JointState::ConstPtr& msg) 
{
  position_moteur_1 = msg->position[0];
  position_moteur_2 = msg->position[1];
  position_moteur_3 = msg->position[2];
  position_moteur_4 = msg->position[3];
  position_moteur_5 = msg->position[4];
  position_moteur_6 = msg->position[5];
  std::cout << "Valeur dans boucle : " << position_moteur_1 << std::endl; 
}

/*
void Lire_Capteur(const std_msgs::Float32::ConstPtr& msg) 
{
  position_moteur = msg->data;
  std::cout << "Valeur dans boucle : " << msg->data << std::endl; 
}
*/
int main(int argc, char** argv) {

    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    ros::Subscriber Mon_Subscriber = nh.subscribe("/valeur_moteur", 100, Lire_Capteur); // Indiquer sur quel topic on subscrit
    ros::Publisher Mon_Publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    
    ros::Rate rate(20);

    // message declarations
    sensor_msgs::JointState joint_state;
    
    std::cout << "Valeur ROS : " << ros::ok() << std::endl;

    while (ros::ok()) 
    {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);

        joint_state.name[0] ="joint_Base_Axe1";
        joint_state.position[0] = position_moteur_1;

        joint_state.name[1] ="joint_Axe1_Axe2";
        joint_state.position[1] = position_moteur_2;

        joint_state.name[2] ="joint_Axe2_Axe3";
        joint_state.position[2] = position_moteur_3;

        joint_state.name[3] ="joint_Axe3_Axe4";
        joint_state.position[3] = position_moteur_4;

        joint_state.name[4] ="joint_Axe4_Axe5";
        joint_state.position[4] = position_moteur_5;

        joint_state.name[5] ="joint_Axe5_Axe6";
        joint_state.position[5] = position_moteur_6; 
        std::cout << "Valeur : " << joint_state.position[1] << std::endl; 

        Mon_Publisher.publish(joint_state);

        rate.sleep();
        ros::spinOnce();
    }

}  