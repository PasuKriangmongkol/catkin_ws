import rospy
import smach
import smach_ros
import time
import os
import pyttsx3
from gtts import gTTS
import speech_recognition as sr
from geometry_msgs.msg import Twist

class WalkToGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        # Move the Turtlebot3 to guest
        # ...
        print("Turtlebot3 reached the guest!")
        return 'succeeded'

class AskForDrink(smach.State):
    def __init__(self, language='en'):
        smach.State.__init__(self, outcomes=['asked', 'failed'])
        self.language = language

    def execute(self, userdata):
        rospy.loginfo("Asking the guest what they would like to drink...")
        # use text-to-speech to ask the guest what they would like to drink
        speech = "What would you like to drink?"
        tts = gTTS(text=speech, lang=self.language)
        tts.save('question.mp3')
        # use a media player to play the speech
        # replace this with the actual implementation of your media player
        os.system('question.mp3')
        r = sr.Recognizer()
        with sr.Microphone() as source:
            audio = r.listen(source)
        try:
            drink = r.recognize_google(audio)
            rospy.loginfo("Guest wants to drink %s." % drink)
            drink = drink.lower()
            userdata.drink_name = drink
            print(drink)
            return 'asked'
        except sr.RequestError as e:
            print("Could not request results; {0}".format(e))
            return 'failed'
        except sr.UnknownValueError:
            print("unknown error occurred")
            return 'failed'

class GoToHost(smach.State):
    def __init__(self, move_pub):
        smach.State.__init__(self, outcomes=['reached'])
        self.host_location = (1, 0)  # fixed location of the host
        self.move_pub = move_pub

    def execute(self, userdata):
        rospy.loginfo("Going to the host...")
        # move to the host using Turtlebot3
        self.move_pub.publish(Twist())
        while not self.at_location(self.host_location):
            rospy.sleep(1)
        self.move_pub.publish(Twist())  # stop the robot's movement
        return 'reached'

    def at_location(self, location):
        # check if the robot is at the given location
        # replace with actual implementation
        return True

class ReportToHost(smach.State):
    def __init__(self, language='en'):
        smach.State.__init__(self, outcomes=['reported'])
        self.language = language

    def execute(self, userdata):
        rospy.loginfo("Reporting to the host...")
        # use text-to-speech to report to the host the guest's drink order
        speech = "The guest wants to drink" + userdata.drink_name + "."
        tts = gTTS(text=speech, lang=self.language)
        tts.save('report.mp3')
        # use a media player to play the speech
        # replace this with the actual implementation of your media player
        os.system('report.mp3')
        return 'reported'

def main():
    rospy.init_node('turtlebot3_barista')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.drink_name = ""

    # Open the container
    with sm:
        # Add the states to the container
        smach.StateMachine.add('WALK_TO_GUEST', WalkToGuest(),
                               transitions={'succeeded': 'ASK_FOR_DRINK'})

        smach.StateMachine.add('ASK_FOR_DRINK', AskForDrink(),
                               transitions={'succeeded': 'TELL_HOST', 'failed': 'failed'})
        
        smach.StateMachine.add('GO_TO_HOST', GoToHost(),
                               transitions={'succeeded': 'REACHED', 'failed': 'failed'})

        smach.StateMachine.add('REPORT_TO_HOST', ReportToHost(),
                               transitions={'succeeded': 'succeeded'})

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()

if _name_ == '_main_':
    main()
