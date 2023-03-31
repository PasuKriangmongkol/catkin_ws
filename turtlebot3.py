import rospy
import smach
import smach_ros
from std_srvs.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import actionlib
from actionlib_msgs.msg import *
import csv
from geometry_msgs.msg import Pose, Point, Quaternion
import pyttsx3
import speech_recognition as sr

class RobotNavigation():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

    def go_to_location(self,x, y ,theta):   
        x,y,theta = float(x), float(y), float(theta)
        q = quaternion_from_euler(0,0,theta) 
        goal = MoveBaseGoal()        
        goal.target_pose.header.frame_id = 'map'        
        goal.target_pose.header.stamp = rospy.get_rostime()        
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
        self.move_base.send_goal(goal)        
        print("Waiting for result")
        success = self.move_base.wait_for_result(rospy.Duration(60))
        print("Return result: ",success)
        state = self.move_base.get_state()        
        result = False        
        if success and state == GoalStatus.SUCCEEDED:            
            # We made it!            
            result = True        
        else:            
            self.move_base.cancel_goal()        
            self.goal_sent = False        
        return result
        
    def shutdown(self):
        stop_goal = MoveBaseGoal()
        self.move_base.send_goal(stop_goal)
        rospy.loginfo("Stop")

class IDE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])

    def execute(self, ud):
        print("Starting robot state")
        rospy.loginfo("Starting robot state")
        S = str(input("Press s to start: "))
        if S == "s":
            return 'success'
        else:
            return 'fail'

class GoToGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'], output_keys=['speech_output'])

    def execute(self, ud):
        rospy.loginfo("Executing state GoToGuest")
        try:
            go_to_guest = RobotNavigation()
            go_to_guest.go_to_location(0,2,0)
        except rospy.ROSInterruptException:
            rospy.loginfo("Exception thrown")
        engine = pyttsx3.init()
        engine.say("What would you like to drink?")
        engine.runAndWait()

        r = sr.Recognizer()
        while True:
            try:
                with sr.Microphone() as source2:
                    print(">>>>>>>>>>>>>> Initializing .....")
                    r.adjust_for_ambient_noise(source2, duration=0.2)

                    print(">>>>>>>>>>>>>> Listing .....")

                    # listens for the user's input
                    audio2 = r.listen(source2)

                    # Using Google to recognize audio
                    MyText = r.recognize_google(audio2)
                    MyText = MyText.lower()
                    print(MyText)

                    ud.speech_output = MyText
                    
                    return 'success'

            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
                return 'fail'

            except sr.UnknownValueError:
                print("Unknown error occurred")
                return 'fail'



class GoToHost(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'], input_keys=['speech_output'])

    def execute(self, ud):
        rospy.loginfo("Executing state GoToHost")
        try:
            go_to_guest = RobotNavigation()
            go_to_guest.go_to_location(0,2,3.94)
        except rospy.ROSInterruptException:
            rospy.loginfo("Exception thrown")

        MyText = ud.speech_output
        RobotText = "He wants to drink " + MyText
        print(RobotText)

        engine = pyttsx3.init()
        engine.say(RobotText)
        engine.runAndWait()

        return 'success'


def RobotState():
    rospy.init_node('robot_state', anonymous=True)
    sm = smach.StateMachine(outcomes=['---finish---'])

    with sm:
        smach.StateMachine.add('IDE', IDE(),
                            transitions={'success': 'GoToGuest', 'fail': 'IDE'},)
        smach.StateMachine.add('GoToGuest', GoToGuest(),
                            transitions={'success': 'GoToHost', 'fail': 'GoToGuest'},
                            remapping={'GoToGuest_Output': 'MyText'})
        smach.StateMachine.add('GoToHost', GoToHost(),
                            transitions={'success': '---finish---', 'fail': 'IDE'})

    outcome = sm.execute()

if __name__ == '__main__':
    print("Starting robot")
    RobotState()
