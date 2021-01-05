#! /usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Byte, Bool, Int16
from geometry_msgs.msg import Twist
from james_docking.msg import dockAction, dockFeedback, dockResult, sensor_state
# from kobuki_msgs.msg import BumperEvent
import time
class jamesDocking(object):

    _dockingfb = dockFeedback()
    _result = dockResult()

    def __init__(self):
        self._dockServer = actionlib.SimpleActionServer('/try_dock', dockAction, self.feedback_callback, False )
        self._dockServer.start()
        self._velPublisher = rospy.Publisher('/base_controller/cmd_vel', Twist, queue_size=10)
        self._irState = sensor_state()
        self._velocity = Twist()
        self._dockState = Byte()
        # self._bumperState = BumperEvent()
        self._irSubscriber = rospy.Subscriber('/dockdata_T', Int16, self.cb_getIR)
        # self._bumperSubscriber = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.cb_getBUMPER)

        # initialize velocity
        self._velocity.linear.x = 0
        self._velocity.linear.y = 0
        self._velocity.linear.z = 0
        self._velocity.angular.x = 0
        self._velocity.angular.y = 0
        self._velocity.angular.z = 0
        
        rospy.loginfo('Server start...')

    def feedback_callback(self, goal):
        # this callback is called when the action server is called.
        # this is the function that runs auto docking.
        self._success = False
        serverCalled = goal.order

        if serverCalled == True:            
            self._dockState = 0
            rospy.loginfo('Goal arrived...')  

        while not self._success:

            if self._dockServer.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                self._velocity.linear.x = 0
                self._velocity.angular.z = 0
                self._velPublisher.publish(self._velocity)
                self._dockServer.set_preempted()
                break            

            if self._dockState == 0:
                rospy.loginfo('Start docking process...')
                self._dockingfb.sequence = "Start docking process..."
                self._dockServer.publish_feedback(self._dockingfb)
                
                if int(bin(self.dockdata & 0b001001000), 2) >= 0b000001000 :
                    self._velocity.angular.z = 0.6
                
                else:
                    self._velocity.angular.z = -0.6            
                
                self._dockState = 1

            elif self._dockState == 1:
                if int(bin(self.dockdata & 0b100111001), 2) >= 0b000000100 :          
                    self._velocity.angular.z = 0
                    self._velPublisher.publish(self._velocity)
                    self._dockingfb.sequence = 'IR transmitter found!...'
                    self._dockServer.publish_feedback(self._dockingfb)                         

                    if int(bin(self.dockdata & 0b000111000), 2) >= 0b000001000 :
                        print("state 1 to 5")
                        self._dockState = 5
                    else :
                        self._dockState = 2         
                
                self._velPublisher.publish(self._velocity)

            elif self._dockState == 2:
                print("state 2")
                if int(bin(self.dockdata & 0b000111000), 2) >= 0b000001000 :
                    self._dockingfb.sequence = 'Center line found...'
                    self._dockServer.publish_feedback(self._dockingfb)                         
                    self._velocity.linear.x = 0
                    self._velPublisher.publish(self._velocity)
                    print("state 2 to 5")
                    self._dockState = 5
                
                elif int(bin(self.dockdata & 0b011000000), 2) >= 0b001000000 :
                        self._dockingfb.sequence = 'Rotating to the center line...'
                        self._dockServer.publish_feedback(self._dockingfb)                         
                        self._velocity.linear.x = 0
                        self._velPublisher.publish(self._velocity)
                        self._dockState = 3

                elif int(bin(self.dockdata & 0b000000110), 2) >= 0b000000010 :
                        self._dockingfb.sequence = 'Rotating to the center line...'
                        self._dockServer.publish_feedback(self._dockingfb)                         
                        self._velocity.linear.x = 0              
                        self._velPublisher.publish(self._velocity)
                        self._dockState = 4

                self._velocity.linear.x = -0.1
                self._velocity.angular.z = 0
                self._velPublisher.publish(self._velocity)

            elif self._dockState == 3:
                print("state 3")
                self._velocity.linear.x = 0
                self._velocity.angular.z = 0.3
                self._velPublisher.publish(self._velocity)
                
                if int(bin(self.dockdata & 0b000111001), 2) >= 0b000000001 :
                    print("state 3 to 5")
                    self._dockState = 5
                    self._dockingfb.sequence = 'Get ready to dock...'
                    self._dockServer.publish_feedback(self._dockingfb)                         
                    
            elif self._dockState == 4:
                print("state 4")
                self._velocity.linear.x = 0
                self._velocity.angular.z = -0.3
                self._velPublisher.publish(self._velocity)
                
                if int(bin(self.dockdata & 0b100111000), 2) >= 0b000001000 :
                    print("state 4 to 5")
                    self._dockState = 5
                    self._dockingfb.sequence = 'Get ready to dock...'
                    self._dockServer.publish_feedback(self._dockingfb)                         

            elif self._dockState == 5:
                    self._velocity.angular.z = 0 
                    self._velPublisher.publish(self._velocity)
                    self._dockingfb.sequence = "Initiate docking..."
                    self._dockServer.publish_feedback(self._dockingfb)
                    self._dockState = 6

            elif self._dockState == 6:
                count = 0

                # if (self._bumperState.bumper == 1) and (self._bumperState.state == 1):
                #     self._velocity.linear.x = 0
                #     self._velocity.angular.z = 0
                #     self._velPublisher.publish(self._velocity)
                #     self._success = True

                if int(bin(self.dockdata & 0b000010000), 2) == 0b000010000 : #   ~|~|~  ~|C|~  ~|~|~
                    self._velocity.linear.x = -0.045 #0.07
                    self.angular_z = 0
                    if int(bin(self.dockdata & 0b000100000), 2) == 0b000100000 : #   ~|~|~  L|C|~  ~|~|~
                        self.angular_z += -0.15
                        print("center & left")
                    if int(bin(self.dockdata & 0b000001000), 2) == 0b000001000 : #   ~|~|~  ~|C|R  ~|~|~
                        self.angular_z += 0.15
                        print("center & left")
                    
                    self._velocity.angular.z = self.angular_z
                    self._velPublisher.publish(self._velocity)

                elif int(bin(self.dockdata & 0b10000001), 2) >= 0b000000001 : #   L|~|~  ~|~|~  ~|~|R
                    self._velocity.linear.x = -0.035
                    self.angular__z = 0
                    if int(bin(self.dockdata & 0b000000001), 2) == 0b000000001 : #   ~|~|~  ~|~|~  ~|~|R
                        print("right")
                        self.angular__z += -0.15
                    if int(bin(self.dockdata & 0b100000000), 2) == 0b100000000 : #   L|~|~  ~|~|~  ~|~|~
                        print("left")
                        self.angular__z += 0.15
                    self._velocity.angular.z = self.angular__z
                    self._velPublisher.publish(self._velocity)


                elif int(bin(self.dockdata & 0b000100000), 2) >= 0b000000001 : #   ~|~|~  L|~|~  ~|~|~
                    self._velocity.linear.x = -0.035
                    self._velocity.angular.z = -0.25
                    self._velPublisher.publish(self._velocity)
                    print("real real real left")

                elif int(bin(self.dockdata & 0b000001000), 2) >= 0b000001000 : #   ~|~|~  ~|~|R  ~|~|~
                    self._velocity.linear.x = -0.035
                    self._velocity.angular.z = 0.25
                    self._velPublisher.publish(self._velocity)            
                    print("real real real right")

                elif int(bin(self.dockdata & 0b001000000), 2) >= 0b001000000 : #   ~|~|R  ~|~|~  ~|~|~
                    self._velocity.linear.x = 0
                    self._velocity.angular.z = 0.7
                    self._velPublisher.publish(self._velocity)     

                elif int(bin(self.dockdata & 0b000000100), 2) >= 0b000000100 : #   ~|~|~  ~|~|~  L|~|~
                    self._velocity.linear.x = 0
                    self._velocity.angular.z = -0.7
                    self._velPublisher.publish(self._velocity)

                else :
                    self._velocity.linear.x = -0.03 #0.03
                    self._velocity.angular.z = 0
                    self._velPublisher.publish(self._velocity)


            if self._success == True:
                self._velocity.linear.x = 0
                self._velocity.angular.z = 0
                self._velPublisher.publish(self._velocity)
                self._result.consequence = self._success
                rospy.loginfo('Successfully docked!')
                self._dockingfb.sequence = "Successfully docked!"
                self._dockServer.publish_feedback(self._dockingfb)
                self._dockServer.set_succeeded(self._result)
                

    def cb_getIR(self, msg):
        self._dockdata = msg
        self.dockdata = self._dockdata.data



    def cb_getBUMPER(self, msg):
        self._bumperState = msg

if __name__ == '__main__':
    rospy.init_node('james_docking')
    jamesDocking()
    rospy.spin()
