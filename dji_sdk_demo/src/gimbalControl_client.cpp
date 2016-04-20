#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;

static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+------------- < Main menu > ----------+\n");
	printf("                    (k)                 \n");
	printf("                     A                  \n");	
	printf("                     |                  \n");
    printf("                     |                  \n");
    printf("                     |                  \n");
    printf(" (h)<------------------------------>(k) \n");
	printf("                     |                  \n");
	printf("                     |                  \n");
    printf("                     |                  \n");
    printf("                     V                  \n");
   	printf("                    (k)                 \n");
    printf("+--------------------------------------+\n");
    printf("input h, j, k and l and then press enter key to move the camera\r\n");
    printf("--------------------------------------\r\n");
    printf("input: ");
}
int main(int argc, char **argv)
{
    int direction;
    int temp32;
    bool valid_flag = false;
    bool err_flag = false;
    bool exit_flag = true;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);
    if(drone->request_sdk_permission_control())
        printf("\n Permission Control Acquired\n");
	
    Display_Main_Menu();
    while(exit_flag)
    {
		ros::spinOnce();
        temp32 = getchar();
        if(temp32 != 10)
        {
            if(valid_flag == false)
            {
                direction = temp32;
                valid_flag = true;
            }
            else
            {
                err_flag = true;
            }
            continue;
        }
        else
        {
            if(err_flag == true)
            {
                printf("input: ERROR\n");
                Display_Main_Menu();
                err_flag = valid_flag = false;
                continue;
            }
        }
        switch(direction)
        {
			case 'h':
				if(drone->gimbal_angle_control(0, 0, 300, 20, 0))
                    printf("\nGimbal yaw has been increased 30º\n");
                sleep(2);
                break;
            case 'l':
                if(drone->gimbal_angle_control(0, 0, -300, 20, 0))
                     printf("\nGimbal yaw has been drecreased 30º\n");
                sleep(2);
                break;
            case 'j':
                if(drone->gimbal_angle_control(0, -300, 0, 20, 0))
                    printf("\nGimbal pitch has been increased 30º");
                sleep(2);
                break;
            case 'k':
                if(drone->gimbal_angle_control(0, 300, 0, 20, 0))
                    printf("\nGimbal yaw has been decreased 30º");
                sleep(2);
                break;
            case 'q':
                exit_flag = false;
                break;
            default:
                break;
        }
        direction = -1;
        err_flag = valid_flag = false;
        Display_Main_Menu();
    }
    return 0;
}
