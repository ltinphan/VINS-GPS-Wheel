#include <vector>
#include <thread>
#include <signal.h>

#include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <custom_msgs/Encoder.h>
// #include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// #include "parameters.h"

const int nDelayTimes = 2;

const double LEFT_D = 0.623479;
const double RIGHT_D = 0.622806;
const double WHEELBASE = 1.52439;
const double ENC_RESOLUTION = 4096;

custom_msgs::Encoder current_enc, last_enc;
double x = 0.0, y = 0.0, theta = 0.0, vx = 0.0, vy = 0.0, vtheta = 0.0;
double lastest_time = 0;

ros::Publisher odomwheel_pub;
ros::Subscriber sub_rawencoder;
// tf::TransformBroadcaster odom_broadcaster;

void rawencoder_callback(const custom_msgs::Encoder::ConstPtr &encoder_msg)
{
    current_enc = *encoder_msg;
    double t = encoder_msg->header.stamp.toSec();
    double dt = t - lastest_time;

    // compute enc vel
    double enc_vel_left = (double)(current_enc.left_encoder - last_enc.left_encoder) / ENC_RESOLUTION * M_PI * LEFT_D / dt;
    double enc_vel_right = (double)(current_enc.right_encoder - last_enc.right_encoder) / ENC_RESOLUTION * M_PI * RIGHT_D / dt;
    double enc_v = 0.5 * (enc_vel_left + enc_vel_right);
    double enc_omega = (enc_vel_right - enc_vel_left) / WHEELBASE;
    // vx = enc_v * cos(theta);
    vx = enc_v;
    // vy = enc_v * sin(theta);
    vy = 0;
    vtheta = enc_omega;
    //compute odometry in a typical way given the velocities of the robot
    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
    double delta_theta = vtheta * dt;

    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    nav_msgs::Odometry odom;
    odom.header.stamp = current_enc.header.stamp;
    odom.header.frame_id = "world";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    // odom.child_frame_id = "odom_wheel";
    odom.twist.twist.linear.x = enc_v * cos(theta);
    odom.twist.twist.linear.y = enc_v * sin(theta);
    odom.twist.twist.angular.z = vtheta;

    //publish the message
    odomwheel_pub.publish(odom);
    lastest_time = t;
    last_enc = current_enc;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odomwheel_node");
    ros::NodeHandle nh("~");
    // ros::Publisher odomwheel_pub = nh.advertise<nav_msgs::Odometry>("odomwheel", 100, true);
    odomwheel_pub = nh.advertise<nav_msgs::Odometry>("odomwheel", 100, true);
    sub_rawencoder = nh.subscribe("/encoder/data_raw", 2000, rawencoder_callback);

    ros::spin();

    return 0;
}