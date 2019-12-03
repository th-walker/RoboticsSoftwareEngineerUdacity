#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
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
    int move_left = 0;
    int move_mid = 0;
    int move_right = 0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // Loop through each pixel in image and check if there is a bright white one
    for (int i = 0: i < img.height * img.step; i++)
    {
        // Identify if the pixel falls in the left, mid, or right side of image
        if (img.data[i] == img.data[white_pixel])
        {
            // pixel left of image
            if (img.height[i] <= (1/3)*img.height) 
            {
                move_left = 1;
                // call drive_bot function
            }

            // pixel mid of image
            if ((1/3)*img.height < img.height[i] <= (2/3)*img.height) 
            {
                move_mid = 1;
                // call drive_bot function
            }

            // pixel right of image
            if ((2/3)*img.height < img.height[i]) 
            {
                move_right = 1;
                // call drive_bot function
            }
        }
        // Request a stop when there is no white ball seen by the camera
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