#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <cmath>
#include <Eigen/Dense> 


// Define global variables
double a; // half-width of lemniscate
double t; // angle parameter of lemniscate
double Ts = 0.005; // angle parameter increment of lemniscate

// ... (other includes and global variables)

// Main function
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "lemniscate_trajectory");
    ros::NodeHandle nh;

    // Get parameters from command line or launch file
    // Get parameters from command line or launch file
    ros::param::get("~a", a); // half-width of lemniscate
    ros::param::get("~Ts", Ts); // angle parameter increment of lemniscate
    
    ros::Rate loop_rate(int(1/ (Ts)));
    t=0;

    // Create a publisher object for the Twist message topic
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("desired_twist", 1000);

    while (ros::ok()) {
        // Calculate desired position and orientation of robot on lemniscate using parametric equations
        ros::Time current_time = ros::Time::now();
        double t_new = t + Ts; //  ROS_TIME

        double x_d = a * sqrt(2) * cos(t_new) / (sin(t_new) * sin(t_new) + 1);
        double y_d = a * sqrt(2) * cos(t_new) * sin(t_new) / (sin(t_new) * sin(t_new) + 1);
        double theta_d = atan2(y_d, x_d);

        double v_x_d = (a * sqrt(2) * cos(t_new) / (sin(t_new) * sin(t_new) + 1));
        double v_y_d = (a * sqrt(2) * cos(t_new) * sin(t_new) / (sin(t_new) * sin(t_new) + 1));


        geometry_msgs::Twist desired_twist;
        desired_twist.linear.x = x_d;
        desired_twist.linear.y = y_d;
        desired_twist.angular.z = theta_d;
        desired_twist.angular.x = v_x_d;
        desired_twist.angular.y = v_y_d;


        twist_pub.publish(desired_twist);

        // Increment the angle parameter of lemniscate for the next iteration
        t += Ts ;

        // Spin once to handle callbacks
        ros::spinOnce();

        // Sleep until next cycle
        loop_rate.sleep();
    }

    return 0;
}

