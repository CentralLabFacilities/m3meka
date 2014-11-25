/* 
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <iostream>

#include <ros/ros.h>
#include "m3/hardware/led_matrix_ec_shm_sds.h"
#include <shm_led_mouth/LEDMatrixCmd.h>

class LEDDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_pub_;

public:
  //! ROS node initialization
  LEDDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_pub_ = nh_.advertise<shm_led_mouth::LEDMatrixCmd>("/led_matrix_command", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveLED()
  {
    char cmd[50];
     shm_led_mouth::LEDMatrixCmd led_matrix_cmd;     
     
    led_matrix_cmd.row.resize(NUM_ROWS);
    
    for (int i = 0; i < NUM_ROWS; i++)
      led_matrix_cmd.row[i].column.resize(NUM_COLS);
    
    led_matrix_cmd.enable = true;
    led_matrix_cmd.header.stamp = ros::Time::now();
    led_matrix_cmd.header.frame_id = "led_matrix_cmd";
    
    for (int i = 0; i < NUM_ROWS; i++)
    {
      for (int j = 0; j < NUM_COLS; j++)
      {
	led_matrix_cmd.row[i].column[j].r = 40;
	led_matrix_cmd.row[i].column[j].b = 20;
	led_matrix_cmd.row[i].column[j].g = 30;
      }
    }
    

    std::cout << "Press any key to cmd LED.\n";
    std::cin.getline(cmd, 50);
    
    
    cmd_pub_.publish(led_matrix_cmd);
    
        
    std::cout << "Press any key to stop LED.\n";
    std::cin.getline(cmd, 50);
    
    led_matrix_cmd.header.stamp = ros::Time::now();
    led_matrix_cmd.enable = false;
    cmd_pub_.publish(led_matrix_cmd);
    
    
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "mouth_driver");
  ros::NodeHandle nh;

  LEDDriver driver(nh);
  driver.driveLED();
}
