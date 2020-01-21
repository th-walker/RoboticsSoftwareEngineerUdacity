#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_bot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Passing velocities to drive robot");
    

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Error check
    ROS_INFO_STREAM(lin_x);
    ROS_INFO_STREAM(ang_z);

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int move_left = 0;
    int move_right = 0;
    int move_forward = 0;

  for (int i = 0; i < img.height * img.step; i+=3) {
        int location = i % (img.width * 3) / 3;
    	if(img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel){
          if(location <= 300){
            move_left +=1;
          }else if(location >= 600){
            move_right +=1;
          }else{
            move_forward +=1;
          }
        }
  }
      
    if(move_left > move_right && move_left > move_forward){
      drive_bot(0.0, -0.5);
    }else if(move_right > move_left && move_right > move_forward){
      drive_bot(0.0, 0.5);
    }else if(move_forward > move_left && move_forward > move_right){
      drive_bot(0.5, 0.0);
    }else{
      drive_bot(0.0, 0.0);
    }
    ros::Duration(3).sleep();
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