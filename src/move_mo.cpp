#include <string>
#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "dynamixel_msgs/MotorStateList.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<iostream>
using namespace std;

class server
{
  public:

    double x_pos;
    double y_pos;
    double x_pos_cam;

    double roll_base, pitch_base, yaw_base;
    double yaw_motor_pos, pitch_motor_pos, roll_motor_pos;

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void motor_states(const dynamixel_msgs::MotorStateList::ConstPtr& msg);
    void pred_pose_velCallback(const nav_msgs::Odometry::ConstPtr& msg);
};


//Reading odometry of the drivetrain
void server::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
x_pos = msg->pose.pose.position.x;
y_pos = msg->pose.pose.position.y;

tf::Quaternion q(
msg->pose.pose.orientation.x,
msg->pose.pose.orientation.y,
msg->pose.pose.orientation.z,
msg->pose.pose.orientation.w);

tf::Matrix3x3 m(q);
m.getRPY(roll_base, pitch_base, yaw_base);
}


//Reading motor values of the neck
void server::motor_states(const dynamixel_msgs::MotorStateList::ConstPtr& msg)
{
yaw_motor_pos = msg->motor_states[0].position;
pitch_motor_pos = msg->motor_states[1].position;
roll_motor_pos = msg->motor_states[2].position;
}

void server::pred_pose_velCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_pos_cam = msg->pose.pose.position.z;
}


int main(int argc, char** argv) {
    server objserver;
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/ropod/odom", 1, &server::OdomCallback, &objserver);
    ros::Subscriber sub3 = n.subscribe("/motor_states/pan_tilt_port", 1, &server::motor_states, &objserver);
    ros::Subscriber sub4 = n.subscribe("/ropod/pred_pose_vel", 1, &server::pred_pose_velCallback, &objserver);
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;

    tf2::Quaternion q_rot_base, q_rot_head;
    ros::Rate loop_rate(200);

    // message declarations
    geometry_msgs::TransformStamped odom_trans, odom_trans2, head_trans, head_trans2;

    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "ropod__base_link";
    
    odom_trans2.header.frame_id = "world";
    odom_trans2.child_frame_id = "base_link2";

    head_trans.header.frame_id = "base_link";
    head_trans.child_frame_id = "head_Link";
    
    head_trans2.header.frame_id = "base_link2";
    head_trans2.child_frame_id = "head_link2";



    while (ros::ok()) {

    //RPY to quaternion with rotation of 90 degrees
    q_rot_base.setRPY(objserver.roll_base, objserver.pitch_base, objserver.yaw_base);
    q_rot_base.normalize();

    // update transform
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x =  objserver.x_pos;
    odom_trans.transform.translation.y = objserver.y_pos;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation.x = 0;
    odom_trans.transform.rotation.y = 0;
    odom_trans.transform.rotation.z = q_rot_base[2];
    odom_trans.transform.rotation.w = q_rot_base[3];
    
    odom_trans2.header.stamp = ros::Time::now();
    odom_trans2.transform.translation.x = -6.5;
    odom_trans2.transform.translation.y = objserver.x_pos_cam;
    odom_trans2.transform.translation.z = 0.0;

    odom_trans2.transform.rotation.x = 0;//0.707388269167200;
    odom_trans2.transform.rotation.y = 0;
    odom_trans2.transform.rotation.z = -0.7068252;
    odom_trans2.transform.rotation.w = 0.7073883;//-0.706825181105366;




    //declaring head rotation variables
    double yaw_head, pitch_head, roll_head;
    //Encoder pulses to radians
    yaw_head = (objserver.yaw_motor_pos - 2047) /651.74;
    pitch_head = (objserver.pitch_motor_pos - 3765) / 651.74;
    roll_head = 0;

    //Converting euler to quaternion (pitch and roll are switched!)
    q_rot_head.setRPY(M_PI/2, 0, M_PI);
    q_rot_head.normalize();
    head_trans.header.stamp = ros::Time::now();
    head_trans.transform.translation.y = 0.05; //fixed offset between base and head
    head_trans.transform.translation.z = 0.2; //fixed offset between base and head
    head_trans.transform.rotation.x = q_rot_head[0];
    head_trans.transform.rotation.y = q_rot_head[1];
    head_trans.transform.rotation.z = q_rot_head[2];
    head_trans.transform.rotation.w = q_rot_head[3];
    
    head_trans2.header.stamp = ros::Time::now();
    head_trans2.transform.translation.y = 0.05;
    head_trans2.transform.translation.z = 0.2;
    head_trans2.transform.rotation.x = 0;
    head_trans2.transform.rotation.y = 0;
    head_trans2.transform.rotation.z = 0;
    head_trans2.transform.rotation.w = 1;


     //send the joint state and transform
    broadcaster.sendTransform(odom_trans);
    broadcaster.sendTransform(head_trans);
    broadcaster.sendTransform(odom_trans2);
    broadcaster.sendTransform(head_trans2);


    ros::spinOnce();
    loop_rate.sleep();
    }


    return 0;
}
