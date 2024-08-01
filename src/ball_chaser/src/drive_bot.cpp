#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBot{
    private:
        ros::NodeHandle n_;
        ros::Publisher motor_command_publisher_;
        ros::ServiceServer service_;

    public:
        DriveBot(ros::NodeHandle& n) : n_(n){
            // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist 
            // on the robot actuation topic with a publishing queue size of 10
            motor_command_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

            // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
            service_ = n_.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
            ROS_INFO("Ready to sent chaser commands");
        }
        
        bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){

            if(ros::ok()){
                // Create a motor_command object of type geometry_msgs::Twist
                geometry_msgs::Twist motor_command;
                // Set wheel velocities, forward [0.5, 0.0]
                motor_command.linear.x = req.linear_x;
                motor_command.angular.z = req.angular_z;
                // Publish angles to drive the robot
                motor_command_publisher_.publish(motor_command);
            }else{
                ROS_INFO("Error in ros::ok() if condition");
                return false;
            }
            ROS_INFO("DriveToTarget received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);
            return true;
        }


};

int main(int argc, char** argv){
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    DriveBot drive_bot(n);
    ros::spin();

    return 0;
}