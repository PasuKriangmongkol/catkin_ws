#!/usr/bin/env python3
import rospy
import smach
import smach_ros
from std_srvs.srv import *
from your_smach.srv import GoToPosition, GoToPositionRequest

class IDLE(smach.state) :
    def __init__(self) -> None:
        super().__init__(self,outcomes=['success', 'fail']):
        if 

class H2G(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.dosth_ser = rospy.ServiceProxy('robot/do_sth', Empty)
    
    def execute(self, ud):
        rospy.loginfo("Executing state H2G")
        self.dosth_ser()
        return 'success'

class G2H(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.gotopos = rospy.ServiceProxy('robot/go_to_position', GoToPosition)
    
    def execute(self, ud):
        rospy.loginfo("Executing state G2H)
        req = GoToPositionRequest()
        req.position_name = 'home'
        self.gotopos(req)
        return 'success' 

class RobotState(object):
    def __init__(self) -> None:
        rospy.init_node('robot_state', anonymous=True)
        sm = smach.StateMachine(outcomes=['---finish---'])

        with sm:
            smach.StateMachine.add('IDLE', IDLE(), 
                                transitions={'success': 'start', 'fail': 'IDLE'})
            smach.StateMachine.add('H2G', H2G(), 
                                transitions={'success':'G2H', 'fail':'H2G'})
            smach.StateMachine.add('G2H', G2H(), 
                                transitions={'success':'---finish---', 'fail':'G2H'})
            smach.StateMachine.add('DoSth2', DoSth(), 
                                transitions={'success':'---finish---', 'fail':'DoSth2'})
        outcome = sm.execute()

if __name__ == "__main__":
    RobotState()
