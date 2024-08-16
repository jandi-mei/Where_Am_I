#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv))
    {
        ROS_ERROR("There is error requesting for drive_bot service");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int white_pixel = 255;
    int ball_position = -1; // -1 means no ball detected

    // Calculate the total number of pixels
    int total_pixels = img.height * img.width;

    // Loop through each pixel in the image
    for (int i = 0; i < total_pixels; i++)
    {
        // Calculate the index for R, G, B channels
        int red_index = i * 3;
        int green_index = i * 3 + 1;
        int blue_index = i * 3 + 2;

        // Check if the pixel is white
        if (img.data[red_index] == white_pixel && 
            img.data[green_index] == white_pixel && 
            img.data[blue_index] == white_pixel)
        {
            // Determine the position of the white ball
            if (i % img.width < img.width / 3) // Left
            {
                ball_position = 0; // Left
            }
            else if (i % img.width < 2 * img.width / 3) // Middle
            {
                ball_position = 1; // Middle
            }
            else // Right
            {
                ball_position = 2; // Right
            }
            break; // Exit the loop after finding the first white pixel
        }
    }

    if(ball_position == -1)
    {
        drive_robot(0.0, 0.0);
    }
    else if(ball_position== 0)
    {
        drive_robot(0.0, 0.5);
    }
    else if(ball_position== 1)
    {
        drive_robot(0.5, 0.0);
    }
    else
    {
        drive_robot(0.0, -0.5);
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