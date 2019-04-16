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
import time
import rclpy
from geometry_msgs.msg import Twist

gain_t =0.03
gama2=1.25
c = 0.35
kbeta=1.3
k1=8.5
msg = """
Change your paremeter!
---------------------------
Moving around:
   q    w    e    r    t
   a    s    d    f    g
   z

q/a : increase/decrease gain_t (default : 0.127)
w/s : increase/decrease gama2  (default : 1.25)
e/d : increase/decrease c      (default : 0.35)
r/f : increase/decrease kbeta  (default : 1.3)
t/g : increase/decrease k1     (default : 8.5)

space key, z : reset paremeter

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


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    node = rclpy.create_node('gain_par')
    pub = node.create_publisher(
        Twist, 'gain_par')

    status = 0
    gain_t =0.115000
    gama2=0.100000
    gama1=2.2
    c = 0.35
    kbeta=1.3
    k1=8.5

    try:
        print(msg)
        while(1):
            key = getKey(settings)
            if key == 'q' :
                gain_t +=0.001 
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'a' :
                gain_t -=0.001
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'w' :
                gama2+=0.01
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 's' :
                gama2-=0.01
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'e' :
                c+=0.01
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'd' :
                c-=0.01
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'r' :
                kbeta+=0.01
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'f' :
                kbeta-=0.01
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 't' :
                k1+=0.01 
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'g' :
                k1-=0.01
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'y' :
                gama1+=0.01 
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == 'h' :
                gama1-=0.01
                status = status + 1
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            elif key == ' ' or key == 'z' :
                gain_t =0.115000
                gama1=2.2
                gama2=0.100000
                c = 0.35
                kbeta=1.3
                k1=8.5 
                print("currently:\tgain_t=%f\tgama2=%f\tc=%f\tkbeta=%f\tk1=%f\tgama1=%f\n " % (gain_t,gama2,c,kbeta,k1,gama1))
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()
            twist.linear.x=gain_t; twist.linear.y=gama2;  twist.linear.z=c;
            twist.angular.x = kbeta; twist.angular.y = k1; twist.angular.z = gama1; 

            pub.publish(twist)
            time.sleep(0.1)
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
