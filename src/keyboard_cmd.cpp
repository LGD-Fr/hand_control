#include <termios.h>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <locale.h>
#include <ncurses.h>
#include "display.h"

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

class Run
{
  private:
    std_msgs::Empty empty;
    ros::NodeHandle n;
    ros::Rate loop_rate;
    ros::Publisher cmd, pub_takeoff, pub_land, pub_reset;
    void land()
      { pub_land.publish(empty); };
    void takeoff()
      { pub_takeoff.publish(empty); };
    void reset()
      { pub_reset.publish(empty); };
    float x_speed, y_speed, z_speed, turn;
    Curses term;
    void print_speed() { ; };

  public:
    Run() :
      loop_rate(30),
      x_speed(0.2),
      y_speed(0.3),
      z_speed(0.5),
      turn(0.5) {
        cmd =  n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
        pub_takeoff =  n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
        pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
        pub_reset = n.advertise<std_msgs::Empty>("/ardrone/reset", 1);
      }

    void operator()()
    {
      while (ros::ok())
      {
        geometry_msgs::Twist::Ptr msg(new geometry_msgs::Twist());
        msg->linear.x = msg->linear.y = msg->linear.z =
          msg->angular.x = msg->angular.y = msg->angular.z = 0.;

        char c = getch();

        switch(c)
        {
          case 'k' :
            {// hover
              cmd.publish(msg);
              break;
            }
          case 'i' :
            {// forward
              msg->linear.x = x_speed;
              cmd.publish(msg);
              break;
            }
          case ';' :
            {// backward
              msg->linear.x = -x_speed;
              cmd.publish(msg);
              break;
            }
          case 'h' :
            {//translate left
              msg->linear.y = -y_speed;
              cmd.publish(msg);
              break;
            }
          case 'm' :
            {//translate right
              msg->linear.y = y_speed;
              cmd.publish(msg);
              break;
            }
          case 'j' :
            {//rotate left
              msg->angular.z = turn;
              cmd.publish(msg);
              break;
            }
          case 'l' :
            {//rotate right
              msg->angular.z = -turn;
              cmd.publish(msg);
              break;
            }
          case 'u' :
            {//turn left
              msg->angular.z = turn;
              msg->linear.x = x_speed;
              cmd.publish(msg);
              break;
            }
          case 'o' :
            {//turn right
              msg->angular.z = -turn;
              msg->linear.x = x_speed;
              cmd.publish(msg);
              break;
            }
          case ',' :
            {//turn left backward
              msg->angular.z = turn;
              msg->linear.x = -x_speed;
              cmd.publish(msg);
              break;
            }
          case ':' :
            {//turn right backward
              msg->angular.z = -turn;
              msg->linear.x = -x_speed;
              cmd.publish(msg);
              break;
            }
          case 'y' :
            {//up
              msg->linear.z = z_speed;
              cmd.publish(msg);
              break;
            }
          case 'n' :
            {//down
              msg->linear.z = -z_speed;
              cmd.publish(msg);
              break;
            }
          case 't' :
            {//takeoff
              takeoff();
              break;
            }
          case 'b' :
            {//land
              land();
              break;
            }
          case 'g' :
            {//reset
              reset();
              break;
            }
          case 'a' :
            {// + x_speed
              x_speed *= 1.1;
              print_speed();
              break;
            }
          case 'w' :
            {// - x_speed
              x_speed *= 0.9;
              print_speed();
              break;
            }
          case 'z' :
            {// + y_speed
              y_speed *= 1.1;
              print_speed();
              break;
            }
          case 'x' :
            {// - y_speed
              y_speed *= 0.9;
              print_speed();
              break;
            }
          case 'e' :
            {// + z_speed
              z_speed *= 1.1;
              print_speed();
              break;
            }
          case 'c' :
            {// - z_speed
              z_speed *= 0.9;
              print_speed();
              break;
            }
          case 'r' :
            {// + turn speed
              turn *= 1.1;
              print_speed();
              break;
            }
          case 'v' :
            {// - turn speed
              turn *= 0.9;
              print_speed();
              break;
            }
          default :
            break;
        } // switch
        ros::spinOnce();
        loop_rate.sleep();
      } // while
    } // run

}; // class

int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "keyboard_cmd");
  Curses terminal;
  terminal.print_kbd();
  for(;;) ;
  return 0;
}

