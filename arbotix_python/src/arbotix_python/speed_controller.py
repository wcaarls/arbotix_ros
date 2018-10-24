#!/usr/bin/env python

"""
  speed_controller.py - controller for a kinematic chain
  Copyright (c) 2018 Wouter Caarls.  All right reserved.
  Parts copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy

from arbotix_msgs.msg import *
from diagnostic_msgs.msg import *

from ax12 import *
from controllers import *

class SpeedController(Controller):
    """ A controller for joint chains, exposing a speed command. """

    def __init__(self, device, name):
        Controller.__init__(self, device, name)
        self.interpolating = 0

        # parameters: rates and joints
        self.rate = rospy.get_param('~controllers/'+name+'/rate',50.0)
        self.joints = rospy.get_param('~controllers/'+name+'/joints')
        self.index = rospy.get_param('~controllers/'+name+'/index', len(device.controllers))
        for joint in self.joints:
            self.device.joints[joint].controller = self

        # speed
        rospy.Subscriber(self.name+'/command', JointVelocities, self.commandCb)
        self.executing = False

        rospy.loginfo("Started SpeedController ("+self.name+"). Joints: " + str(self.joints) + " on C" + str(self.index))

    def startup(self):
        pass

    def commandCb(self, msg):
        executing = False
        
        writes = []
        
        for i in range(len(msg.joint_names)):
          joint = self.device.joints[msg.joint_names[i]]
          vel = msg.joint_velocities[i]
          if int(joint.speedToTicks(abs(vel))) > 0:
            executing = True
            if vel > 0:
              pos = joint.max_angle
            else:
              pos = joint.min_angle
          else:
            pos = joint.position
            
          pos = int(joint.angleToTicks(pos))
          vel = int(joint.speedToTicks(abs(vel)))
          
          writes.append([joint.id, pos%256, pos>>8, vel%256, vel>>8])
          
        joint.device.syncWrite(P_GOAL_POSITION_L, writes)
        self.executing = executing

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.executing

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if self.active():
            msg.values.append(KeyValue("State", "Active"))
        else:
            msg.values.append(KeyValue("State", "Not Active"))
        return msg
