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

double x, y, theta; // current position and orientation of robot
double x_goal, y_goal;
double x_e, y_e, theta_e; // errors between desired and current position and orientation
double linear_vel, angular_vel; // linear and angular velocity commands for robot
double Vmax = 100; //Max velocity for the robot
double Vtot; //Max velocity for the robot
double d;
double u1, u2;
double t_max = 360;

// PID parameters
double kp = 11.0;
double kp1 = 1.0;
double kp2 = 1.0;
double err = 0.3;
std::deque<std::pair<double,double>> path;



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
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta); // convert quaternion to Euler angles
}

void path_gen(double step)
{
    double t = 0;
    while (true)
    {
        double x = a * sqrt(2) * cos(t) / (sin(t) * sin(t) + 1);
        double y = a * sqrt(2) * cos(t) * sin(t) / (sin(t) * sin(t) + 1);
        path.push_back(std::make_pair(x, y));
        t = t + step;
    }
}

void path_gen(double step, double total_duration) {
    double t = 0;
    double max_t = total_duration; // Define the total duration for path generation
    while (t <= max_t) { // Check the duration to generate the path
        double x_d = a * sqrt(2) * cos(t) / (sin(t) * sin(t) + 1);
        double y_d = a * sqrt(2) * cos(t) * sin(t) / (sin(t) * sin(t) + 1);
        path.push_back(std::make_pair(x_d, y_d));
        t = t + step;
    }
}

std::pair<double, double> traj_controller(double x_goal, double y_goal, double vx=0, double vy=0)
{
    double vx_d = 0; 
    double vy_d = 0;
    // auto next_point = path.front();
    // double x_d_1 = next_point.first;
    // double y_d_1 = next_point.second;
    // vx_d = kp1*(x_d_1 - x_goal) / Ts;
    // vy_d = kp2*(y_d_1 - y_goal) / Ts;
    double kerr = sqrt(pow((x_goal - x),2) + pow((y_goal - y),2));
    double u1 = (vx_d/(100*kerr)) + kp * (x_goal - x);
    double u2 = (vy_d/(100*kerr)) * (y_goal - y);

    
    // feedback linearization
    Eigen::Matrix2d A;
    A << cos(theta), -d*sin(theta),
            sin(theta), d*cos(theta);
    
    Eigen::Vector2d vw = A.inverse() * Eigen::Vector2d(u1,u2);
    
    return std::make_pair(vw[0], vw[1]);
}

// Main function
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "lemniscate_controller");
    ros::NodeHandle nh;

    // Get parameters from command line or launch file
    ros::param::get("~a", a); // half-width of lemniscate
    ros::param::get("~kp", kp); // half-width of lemniscate
    ros::param::get("~kp1", kp1); // half-width of lemniscate
    ros::param::get("~kp2", kp2); // half-width of lemniscate
    ros::param::get("~err", err); // half-width of lemniscate
    ros::param::get("~d", d); // half-width of lemniscate

    // Create a subscriber object for odometry topic
    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);

    // Create a publisher object for command velocity topic
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    Ts = 0.05;
    path_gen(Ts, t_max);

    // double t = 0;
    // double max_t = t_max; // Define the total duration for path generation
    // while (t <= max_t) { // Check the duration to generate the path
    //     std::tie(x_goal,y_goal) = path.front();
    //     t = t + Ts;
    //     path.pop_front();
    //     ROS_INFO("x_goal: %f y_goal: %f", x_goal, y_goal);
    // }


    
    // Set loop rate to 10 Hz
    ros::Rate loop_rate(int(1/Ts));

    bool start = 0;

    while (ros::ok()) {
        ROS_INFO("robot.x: %f robot.y: %f", x, y);
        ROS_INFO("err: %f", sqrt(pow((x_goal - x),2) + pow((y_goal - y),2)));

        // Increment angle parameter of lemniscate
        if (sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal)) < err)
            start = true;
        
        if(start){
            std::tie(x_goal,y_goal) = path.front();
            path.pop_front();
            ROS_INFO("x_goal: %f y_goal: %f", x_goal, y_goal);
        }
        // }
        
        auto [v,w] = traj_controller(x_goal,y_goal);

        // Apply PID controllers to errors and generate velocity commands for robot
        linear_vel = v;
        angular_vel = w;

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
