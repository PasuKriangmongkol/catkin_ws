#!/usr/bin/env python3
import rospy
import smach
import smach_ros
from std_srvs.srv import *
from your_smach.srv import GoToPosition, GoToPositionRequest
import pyttsx3
import speech_recognition as sr

class IDLE(smach.state) :
    def __init__(self) -> None:
        super().__init__(self,outcomes=['success', 'fail']):

    def execute(self, ud):
        rospy.loginfo("Starting robot state")
        S = str(input("Press S to start: "))
        if S == "S":
            return 'success'
        else:
            return 'fail'



class GoToGuest(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        
    def execute(self, ud):
        rospy.loginfo("Executing state GoToGuest")
        # Navigation code #
        engine = pyttsx3.init()
        engine.say("What would you like to drink")
        engine.runAndWait()

        r = sr.Recognizer()
        while(1):

            try:
                with sr.Microphone() as source2:
                    print(">>>>>>>>>>>>>> Initializing .....")
                    r.adjust_for_ambient_noise(source2, duration=0.2)
            
                    print(">>>>>>>>>>>>>> Listing .....")

                    # listens for the user's input
                    audio2 = r.listen(source2)

                    # Using google to recognize audio
                    MyText = r.recognize_google(audio2)
                    MyText = MyText.lower()

                    print(MyText) 

                    Mytext = str(MyText)

                    return 'success'
                    

            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
                return 'fail'

            except sr.UnknownValueError:
                print("unknown error occurred")
                return 'fail'



class GoToHost(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
    
    def execute(self, ud):
        rospy.loginfo("Executing state GoToHost")

        return 'success' 

class RobotState(object):
    def __init__(self) -> None:
        rospy.init_node('robot_state', anonymous=True)
        sm = smach.StateMachine(outcomes=['---finish---'])

        with sm:
            smach.StateMachine.add('IDLE', IDLE(), 
                                transitions={'success': 'GoToGuest', 'fail': 'IDLE'})
            smach.StateMachine.add('GoToGuest', GoToGuest(), 
                                transitions={'success':'GoToHost', 'fail':'GoToGuest'})
            smach.StateMachine.add('GoToHost', GoToHost(), 
                                transitions={'success':'---finish---', 'fail':False})
        outcome = sm.execute()

if __name__ == "__main__":
    RobotState()
