#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
|⇑ y|↖ u|↑ i|↗ o|
|⇐ h|← j|  k|→ l|⇒ m|
|⇓ n|↙ ,|↓ ;|↘ :| 

a/w : increase/decrease max speeds by 10%
z/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
    #   x th y z
    'i':(1,0,0,0),
    'o':(1,-1,0,0),
    'j':(0,1,0,0),
    'l':(0,-1,0,0),
    'u':(1,1,0,0),
    ';':(-1,0,0,0),
    ':':(-1,1,0,0),
    ',':(-1,-1,0,0),
    'h':(0,0,-1,0),
    'm':(0,0,1,0),
    'y':(0,0,0,1),
    'n':(0,0,0,-1),
         }

speedBindings={
    'a':(1.1,1.1),
    'w':(.9,.9),
    'z':(1.1,1),
    'x':(.9,1),
    'e':(1,1.1),
    'c':(1,.9),
        }

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

speed = .5
turn = 1

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher('cmd_vel', Twist)
  rospy.init_node('teleop_twist_keyboard_azerty')
  x = 0
  th = 0
  y = 0
  z = 0
  status = 0
  try:
    print msg
    print vels(speed,turn)
    while(1):
      key = getKey()
      if key in moveBindings.keys():
        x = moveBindings[key][0]
        th = moveBindings[key][1]
        y = moveBindings[key][2]
        z = moveBindings[key][3]
      elif key in speedBindings.keys():
        speed = speed * speedBindings[key][0]
        turn = turn * speedBindings[key][1]

        print vels(speed,turn)
        if (status == 14):
          print msg
        status = (status + 1) % 15
      else:
        x = 0
        th = 0
        y = 0
        z = 0
        if (key == '\x03'):
          break
      twist = Twist()
      twist.linear.x = x*speed
      twist.linear.y = y*speed
      twist.linear.z = z*speed
      twist.angular.x = 0; twist.angular.y = 0;
      twist.angular.z = th*turn
      pub.publish(twist)
  except:
    print e
  finally:
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

