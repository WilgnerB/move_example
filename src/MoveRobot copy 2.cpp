#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <cmath>
#include <Eigen/Dense> 


// Define constants
#define PI 3.14159265

// Define global variables
double a; // half-width of lemniscate
double t; // angle parameter of lemniscate
double Ts; // angle parameter of lemniscate
double x_d, y_d, theta_d; // desired position and orientation of robot on lemniscate
double x_d_1, y_d_1, theta_d_1; // desired position and orientation of robot on lemniscate
double v_x_d, v_y_d, v_theta_d; // desired position and orientation of robot on lemniscate
double x, y, theta; // current position and orientation of robot
double x_e, y_e, theta_e; // errors between desired and current position and orientation
double linear_vel, angular_vel; // linear and angular velocity commands for robot
double Vmax = 100; // Max velocity for the robot
double Vtot; // Max velocity for the robot
double d;
double u1, u2; // PID parameters
double kp = 11.0;
double kp1 = 1.0;
double kp2 = 1.0;
double err = 0.3; // do not modify

// Odometry callback function
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Get current position of robot
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    // Get current orientation of robot
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta); // convert quaternion to Euler angles
}

// Main function
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "lemniscate_controller");
    ros::NodeHandle nh;

    // Get parameters from command line or launch file
    ros::param::get("~a", a); // half-width of lemniscate
    ros::param::get("~kp", kp);
    ros::param::get("~kp1", kp1);
    ros::param::get("~kp2", kp2);
    ros::param::get("~err", err);
    ros::param::get("~d", d);

    // Create a subscriber object for odometry topic
    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);

    // Create a publisher object for command velocity topic
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // Initialize angle parameter of lemniscate
    t = 0;
    Ts = 0.05;


    // Set loop rate to 10 Hz
    ros::Rate loop_rate(int(1 / Ts));
    int i=0;
    while (ros::ok()) {
        // Calculate desired position and orientation of robot on lemniscate using parametric equations

        x_d = a * sqrt(2) * cos(t) / (sin(t) * sin(t) + 1);
        y_d = a * sqrt(2) * cos(t) * sin(t) / (sin(t) * sin(t) + 1);
        theta_d = atan2(y_d, x_d);

        x_d_1 = a * sqrt(2) * cos(t + Ts) / (sin(t + Ts) * sin(t + Ts) + 1);
        y_d_1 = a * sqrt(2) * cos(t + Ts) * sin(t + Ts) / (sin(t + Ts) * sin(t + Ts) + 1);
        theta_d_1 = atan2(y_d_1, x_d_1);

        v_x_d = (x_d_1 - x_d) / Ts;
        v_y_d = (y_d_1 - y_d) / Ts;

        double kerr = sqrt(pow((x_goal - x),2) + pow((y_goal - y),2));
        u1 = (vx_d/(100*kerr)) + kp * (x_goal - x);
        u2 = (vy_d/(100*kerr)) * (y_goal - y);

        
        // Vtot = sqrt(pow(kp1 * u1, 2) + pow(kp2 * u2, 2));
        // if (Vtot >= Vmax) {
        //     u1 = u1 * Vmax / Vtot;
        //     u2 = u2 * Vmax / Vtot;
        // }

    
        // feedback linearization
        Eigen::Matrix2d A;
        A << cos(theta), -d*sin(theta),
                sin(theta), d*cos(theta);
        
        Eigen::Vector2d vw = A.inverse() * Eigen::Vector2d(u1,u2);
        

        ROS_INFO("x_d: %f y_d: %f", x_d, y_d);
        ROS_INFO("x_robot: %f y_robot: %f", x, y);
        ROS_INFO("t: %f err: %f", t, sqrt(pow((x_d - x), 2) + pow((y_d - y), 2)));

        //if (sqrt(pow((x_d - x), 2) + pow((y_d - y), 2)) < err) {
        t += Ts;

        //}

        linear_vel = vw[0];
        angular_vel = vw[1];

        // Publish velocity commands to command velocity topic
        geometry_msgs::Twist msg;
        msg.linear.x = linear_vel;
        msg.angular.z = angular_vel;
        pub.publish(msg);

        // Spin once to handle callbacks
        ros::spinOnce();

        // Sleep until next cycle
        loop_rate.sleep();
    }

    return 0;
}