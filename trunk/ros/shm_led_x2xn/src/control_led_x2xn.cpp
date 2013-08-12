/*
Copyright  2008, Meka Robotics
All rights reserved.
http://mekabot.com

Redistribution and use in source and binary forms, with or without
modification, are permitted. 


THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>

#include <ros/ros.h>
#include "m3/hardware/ledx2xn_ec_shm_sds.h"
#include <shm_led_x2xn/LEDX2XNCmd.h>

class X2XNDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_pub_;

public:
  //! ROS node initialization
  X2XNDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_pub_ = nh_.advertise<shm_led_x2xn::LEDX2XNCmd>("/led_x2xn_command", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveX2XN()
  {
    char cmd[50];
     shm_led_x2xn::LEDX2XNCmd led_x2xn_cmd;     
     
    led_x2xn_cmd.branch_a.resize(NUM_PER_BRANCH);
    led_x2xn_cmd.branch_b.resize(NUM_PER_BRANCH);
        
    led_x2xn_cmd.enable_a = true;
    led_x2xn_cmd.enable_b = true;
    led_x2xn_cmd.header.stamp = ros::Time::now();
    led_x2xn_cmd.header.frame_id = "led_x2xn_cmd";
    
    for (int i = 0; i < NUM_PER_BRANCH; i++)
    {     
	led_x2xn_cmd.branch_a[i].r = 40;
	led_x2xn_cmd.branch_a[i].b = 20;
	led_x2xn_cmd.branch_a[i].g = 30;     

	led_x2xn_cmd.branch_b[i].r = 40;
	led_x2xn_cmd.branch_b[i].b = 20;
	led_x2xn_cmd.branch_b[i].g = 30;     

    }
    

    std::cout << "Press any key to cmd LED.\n";
    std::cin.getline(cmd, 50);
    
    
    cmd_pub_.publish(led_x2xn_cmd);
    
        
    std::cout << "Press any key to stop LED.\n";
    std::cin.getline(cmd, 50);
    
    led_x2xn_cmd.header.stamp = ros::Time::now();
    led_x2xn_cmd.enable_a = false;
    led_x2xn_cmd.enable_b = false;
    cmd_pub_.publish(led_x2xn_cmd);
    
    
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "x2xn_driver");
  ros::NodeHandle nh;

  X2XNDriver driver(nh);
  driver.driveX2XN();
}
