#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage{
    private:
        ros::NodeHandle n_;
        ros::ServiceClient client_;
        ros::Subscriber sub1_;
    public:

        ProcessImage(ros::NodeHandle& n) : n_(n){
            // Define a client service capable of requesting services from command_robot
            client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
            // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
            sub1_ = n_.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
        }

        // This function calls the command_robot service to drive the robot in the specified direction
        void drive_robot(float lin_x, float ang_z) {
            // TODO: Request a service and pass the velocities to it to drive the robot
            ball_chaser::DriveToTarget srv;
            srv.request.linear_x = (float)lin_x;
            srv.request.angular_z = (float)ang_z;
            // Call the safe_move service and pass the requested joint angles
            if(!client_.call(srv)){
                ROS_ERROR("Failed to call service drive_robot");
            }
        }

        // This callback function continuously executes and reads the image data
        void process_image_callback(const sensor_msgs::Image img){
            int white_pixel = 255;
            int column_index = 0;
            bool found_ball = false;
            /*
            float left_max = img.step*(1/3); //800;
            float right_min = img.step*(2/3); //1600;
            */

            for(int i = 0; i < img.height * img.step; i++){
                // Check if is white
                if ((img.data[i] == 255) && (img.data[i+1] == 255) && (img.data[i+2] == 255)){
                    // Get the white position
                    column_index = i % img.step;
                    ROS_INFO("Column_index:%1.2f ", (float)column_index);
                    if(column_index < img.step/3){
                        // Left Side of Camera
                        drive_robot(0.5,1);
                    }else if(column_index < (img.step/3 * 2)){
                        // Middle side of camera
                        drive_robot(1, 0.0);
                    }else{
                        // Right side of camera
                        drive_robot(0.5,-1);
                    }
                    found_ball = true;
                    break;
                }
            }

            if(found_ball == false){
                // Turns around util finds it
                drive_robot(0,0.3);
            }
        }

};



int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ProcessImage process_image(n);

    // Handle ROS communication events
    ros::spin();

    return 0;
}