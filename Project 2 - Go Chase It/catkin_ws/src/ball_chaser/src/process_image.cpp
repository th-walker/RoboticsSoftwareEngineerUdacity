#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_bot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Passing velocities to drive robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int img_left = img.step/3;
    int img_mid = 2*img.step / 3;

    for (int i = 0; i < img.height * img.step; i++)
    {
        // Identify if the pixel falls in the left, mid, or right side of image
        if (img.data[i] == img.data[white_pixel])
        {
            ROS_INFO_STREAM("Ball detected");

            // pixel left of image
            if (img.step <= img_left) 
            {
                ROS_INFO_STREAM("Move robot left");
                // call drive_bot function and pass velocities
                drive_bot(0, 5);
            }

            // pixel mid of image
            else if (img_left < img.step <= img_mid) 
            {
                ROS_INFO_STREAM("Move robot forward");
                // call drive_bot function and pass velocities
                drive_bot(5, 0);
            }

            // pixel right of image
            else if (img_mid < img.step)
            {
                ROS_INFO_STREAM("Move robot right");
                // call drive_bot function and pass velocities
                drive_bot(0, -5);
            }

            else
            {
                ROS_INFO_STREAM("Error");
            }
            
        }
        // Request a stop when there is no white ball seen by the camera
        ROS_INFO_STREAM("Stop robot");
        drive_bot(0, 0);
    }
    

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}