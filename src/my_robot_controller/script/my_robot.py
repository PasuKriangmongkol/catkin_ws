#!/usr/bin/env python3
import rospy
import smach
import smach_ros
from std_srvs.srv import *
import pyttsx3
import speech_recognition as sr

class IDE(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success', 'fail'])

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
        super().__init__(outcomes=['success', 'fail'])

    def execute(self, ud):
        rospy.loginfo("Executing state GoToGuest")
        # Navigation to GUEST code #
        #------------------------

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

                    return 'success', MyText

            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
                return 'fail'

            except sr.UnknownValueError:
                print("Unknown error occurred")
                return 'fail'

class GoToHost(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success', 'fail'])

    def execute(self, ud):
        rospy.loginfo("Executing state GoToHost")
        # Navigation to HOST code #
        #-------------------------

        MyText = ud.GoToGuest_Output
        RobotText = "He wants to drink " + MyText
        print(RobotText)

        engine = pyttsx3.init()
        engine.say(RobotText)
        engine.runAndWait()

        return 'success'

class RobotState(object):
    def __init__(self):
        rospy.init_node('robot_state', anonymous=True)
        sm = smach.StateMachine(outcomes=['---finish---'])

        with sm:
            smach.StateMachine.add('IDE', IDE(),
                                transitions={'success': 'GoToGuest', 'fail': 'IDE'})
            smach.StateMachine.add('GoToGuest', GoToGuest(),
                                transitions={'success': 'GoToHost', 'fail': 'GoToGuest'},
                                remapping={'GoToGuest_Output': 'MyText'})
            smach.StateMachine.add('GoToHost', GoToHost(),
                                transitions={'success': '---finish---', 'fail': False})

        outcome = sm.execute()

if __name__ == "_main_":
    print("Starting robot")
    RobotState()