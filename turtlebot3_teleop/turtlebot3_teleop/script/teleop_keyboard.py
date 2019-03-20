# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# /* Author: Darby Lim */
import os, sys
import select
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.0025
ANG_DEG_STEP_SIZE = 5

TURTLEBOT3_MODEL = 'burger'

msg = """
Control Your Ballbot!
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

w/x : increase/decrease linear_x velocity (Ballbot : ~ 0.22)
a/d : increase/decrease linear_y velocity (Ballbot : ~ 2.84)
q/w : increase/decrease angular drgree    (0~360)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def makeAngleSimpleProfile(output, input, slop):
    if abs(input-output)>180:
        if input > output:
            output =  output - slop 
            if output <= -180:
                output=output+360
        elif input < output:
            output =  output + slop 
            if output > 180:
                output=output-360
        else:
            output = input
    else:
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if TURTLEBOT3_MODEL == 'burger':
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if TURTLEBOT3_MODEL == 'burger':
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
        

    return vel

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(
        Twist, 'cmd_vel')

    status = 0
    target_linear_x_vel   = 0.0
    target_linear_y_vel   = 0.0
    target_angular_vel  = 0.0
    target_angular_deg  = 0.0
    control_linear_x_vel  = 0.0
    control_linear_y_vel  = 0.0
    control_angular_deg = 0.0

    try:
        print(msg)
        while(1):
            key = getKey(settings)
            if key == 'w' :
                target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print("currently:\tlinear vel_x %s\t " % (target_linear_x_vel))
            elif key == 'x' :
                target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print("currently:\tlinear vel_x %s\t " % (target_linear_x_vel))
            elif key == 'a' :
                target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print("currently:\tlinear vel_y %s\t " % (target_linear_y_vel))
            elif key == 'd' :
                target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print("currently:\tlinear vel_y %s\t " % (target_linear_y_vel))
            elif key == 'q' :
                target_angular_deg = target_angular_deg - ANG_DEG_STEP_SIZE
                if target_angular_deg <=-180:
                    target_angular_deg=360+target_angular_deg
                status = status + 1
                print("currently:\tlinear tar_deg %s\t " % (target_angular_deg))
            elif key == 'e' :
                target_angular_deg = target_angular_deg + ANG_DEG_STEP_SIZE
                if target_angular_deg >180:
                    target_angular_deg=target_angular_deg-360
                status = status + 1
                print("currently:\tlinear tar_deg %s\t " % (target_angular_deg))
            elif key == ' ' or key == 's' :
                target_linear_x_vel   = 0.0
                target_linear_y_vel   = 0.0
                print("currently:\tlinear vel_x %s\t vel_y %s\t " % (target_linear_x_vel,target_linear_y_vel))
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_x_vel = makeSimpleProfile(control_linear_x_vel, target_linear_x_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_linear_y_vel = makeSimpleProfile(control_linear_y_vel, target_linear_y_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_x_vel; twist.linear.y = control_linear_y_vel; twist.linear.z = 0.0

            control_angular_deg = makeAngleSimpleProfile(control_angular_deg, target_angular_deg, (ANG_DEG_STEP_SIZE/4.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_deg

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
