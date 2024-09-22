#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_srvs.srv import Empty, Trigger
from rip_edu_msgs.srv import *
from rip_edu_msgs.msg import *

host_log = ["Phat","milk"]
global guest_logs
global count_guest
global name_of_current_human
global list_of_drinks
global list_of_names
global name_of_current_drink
max_guest = 2

class PrepareReception(smach.State):
    def __init__(self):
        global guest_logs
        global count_guest
        global name_of_current_human
        global list_of_drinks
        global list_of_names
        global name_of_current_drink
        
        list_of_drinks = ["hot chocolate", "coke", "soy", "water", "lime juice", "tea", "coffee", "soda", "milk", "latte"]

        list_of_names = ["julia","emma", "sara", "laura", "susan", "john",
                          "lucas", "william", "kevin", "peter", "robin",
                          "jeffery", "tan", "sarah", "adam"]
        
        guest_logs = []
        name_of_current_human = ""
        name_of_current_drink = ""
        count_guest = 0

        smach.State.__init__(self, outcomes=['prepare_succeeded', 'prepare_failed'])
        rospy.loginfo('Initializing PrepareReception...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        self.detect_human_in_front = '/rip/turtlebot/vision/detect_human_in_front'
        self.empty_seat_detection = '/rip/turtlebot/vision/empty_seat_detection'
        # self.nav_to_goal_xyt = '/rip/turtlebot/navigation/nav_to_goal_xyt'
        self.nav_to_goal = '/rip/turtlebot/navigation/walk_fix'
        
        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.speech_to_text)
        rospy.wait_for_service(self.detect_human_in_front)
        rospy.wait_for_service(self.empty_seat_detection)
        # rospy.wait_for_service(self.nav_to_goal_xyt)
        rospy.wait_for_service(self.nav_to_goal)

        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        self.detect_human_in_front_service = rospy.ServiceProxy(self.detect_human_in_front, DetectHumanInFront)
        self.empty_seat_detect_service = rospy.ServiceProxy(self.empty_seat_detection, EmptySeatDetection)
        # self.navigation_service = rospy.ServiceProxy(self.nav_to_goal_xyt, NavToGoalXYT)
        self.nav_to_goal_service = rospy.ServiceProxy(self.nav_to_goal, NavigationToGoal)
        rospy.loginfo('PrepareReception successfully initialized')

    def execute(self, ud):
        # Perform any initialization if needed
        rospy.loginfo('PrepareReception state executed')
        return 'prepare_succeeded'
    

class Ready(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready_succeeded', 'ready_failed'])
        rospy.loginfo('Initializing Ready state...')

    def execute(self, ud):
        rospy.wait_for_service('/rip/turtlebot/voice/text_to_speech')
        text_to_speech_service = rospy.ServiceProxy('/rip/turtlebot/voice/text_to_speech', TextToSpeech)
        tts_res = text_to_speech_service("I'm ready for the find my mate task")
        if tts_res.success:
            return 'ready_succeeded'
        else:
            return 'ready_failed'

class NavToRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_succeeded', 'nav_failed'])
        rospy.loginfo('Initializing NavToRoom state...')
        self.nav_to_goal = '/rip/turtlebot/navigation/walk_fix'
        rospy.wait_for_service(self.nav_to_goal)
        self.nav_to_goal_service = rospy.ServiceProxy(self.nav_to_goal, NavigationToGoal)

    def execute(self, ud):
        nav_res = self.nav_to_goal_service("counter")
        if nav_res.success:
            return 'nav_succeeded'
        else:
            return 'nav_failed'

class WaitForGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['guest_detected', 'wait_failed'])
        rospy.loginfo('Initializing WaitForGuest state...')
        self.detect_human_in_front = '/rip/turtlebot/vision/detect_human_in_front'
        rospy.wait_for_service(self.detect_human_in_front)
        self.detect_human_in_front_service = rospy.ServiceProxy(self.detect_human_in_front, DetectHumanInFront)
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.loginfo('WaitForGuest state successfully initialized')

    def execute(self, ud):
        rospy.loginfo('Waiting for guest...')
        tts_res = self.text_to_speech_service("Now im ready to start the mission")
        if not tts_res.success:
            return 'wait_failed'
        res = self.detect_human_in_front_service()
        if res.success:
            rospy.loginfo('Guest detected')
            return 'guest_detected'
        return 'wait_failed'

class GreetGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['greet_succeeded', 'greet_failed'])
        rospy.loginfo('Initializing GreetGuest state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.loginfo('GreetGuest state successfully initialized')

    def execute(self, ud):
        rospy.loginfo('Greeting the guest...')
        tts_res = self.text_to_speech_service("Hello! Welcome to our office. How may I assist you today?")
        if tts_res.success:
            return 'greet_succeeded'
        else:
            return 'greet_failed'

class AskForName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['name_received', 'ask_failed'])
        rospy.loginfo('Initializing AskForName state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.speech_to_text)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        rospy.loginfo('AskForName state successfully initialized')

    def execute(self, ud):
        global name_of_current_human
        global list_of_names
        rospy.loginfo('Asking for the guest\'s name...')
        tts_res = self.text_to_speech_service("May I have your name, please?")
        if not tts_res.success:
            return 'ask_failed'

        stt_res = self.speech_to_text_service()
        if stt_res.success:
            rospy.loginfo(f"Guest's name: {stt_res.text}")
            stt_res.text
            for name in list_of_names:
                if stt_res.text == name:
                    name_of_current_human = stt_res.text
                    return 'name_received'
                else:
                    rospy.loginfo("this is not his name")
            tts_res = self.text_to_speech_service("we don't have that's name in list, please try again")
            if not tts_res.success:
                return 'ask_failed'            
        else:
            return 'ask_failed'
        

class FavoriteDrinkCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['favorite_drink_received', 'favorite_drink_failed'])
        rospy.loginfo('Initializing FavoriteDrinkCheck state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.speech_to_text)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        rospy.loginfo('FavoriteDrinkCheck state successfully initialized')

    def execute(self, ud):
        global guest_logs
        global list_of_drinks
        global name_of_current_drink
        rospy.loginfo('Asking for the guest\'s favorite drink...')
        tts_res = self.text_to_speech_service("What is your favorite drink?")
        if not tts_res.success:
            return 'favorite_drink_failed'

        stt_res = self.speech_to_text_service()
        if stt_res.success:
            for drink in list_of_drinks:
                if stt_res.text == drink:
                    rospy.loginfo(f"Favorite drink is {stt_res.text}")
                    # guest_logs[count_guest] = [name_of_current_human,stt_res.text]
                    name_of_current_drink = stt_res.text
                    # guest_logs.append([name_of_current_human,stt_res.text])
                    # rospy.loginfo(guest_logs)
                    return 'favorite_drink_received'
                else:
                    rospy.loginfo("this have not in list")
            tts_res = self.text_to_speech_service("we don't have that's name in list, please try again")
            if not tts_res.success:
                return 'favorite_drink_failed'   
        else:
            return 'favorite_drink_failed'

class GuideToSeat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['guide_succeeded', 'guide_failed'])
        rospy.loginfo('Initializing GuideToSeat state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.nav_to_goal = '/rip/turtlebot/navigation/walk_fix'
        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.nav_to_goal)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.nav_to_goal_service = rospy.ServiceProxy(self.nav_to_goal, NavigationToGoal)
        rospy.loginfo('GuideToSeat state successfully initialized')

    def execute(self, ud):
        rospy.loginfo('Guiding the guest to their seat...')
        tts_res = self.text_to_speech_service("Please follow me, I will take you to the seat")
        if not tts_res.success:
            return 'guide_failed'
        
        nav_res = self.nav_to_goal_service("seat_center")
        if nav_res.success:
            tts_res = self.text_to_speech_service("We have arrived, Please stay on my back until i give you instructions on where to sit")
            if not tts_res.success:
                return 'guide_failed'
            return 'guide_succeeded'
        else:
            return 'guide_failed'

class ProvideInformation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['info_provided', 'info_failed'])
        rospy.loginfo('Initializing ProvideInformation state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.wait_for_service('/rip/turtlebot/navigation/walk_fix')
        self.nav_service = rospy.ServiceProxy('/rip/turtlebot/navigation/walk_fix', NavigationToGoal)
        rospy.loginfo('ProvideInformation state successfully initialized')

    def execute(self, ud):
        global guest_logs
        global count_guest
        global name_of_current_human
        global name_of_current_drink

        rospy.loginfo('Providing information to the guest...')
        tts_res = self.text_to_speech_service(f"Hello everyone, I will present everyone in this room. The host name is {host_log[0]} and favorite drink is {host_log[1]}")
        rospy.loginfo(f"Hello everyone, I will present everyone in this room. The host name is {host_log[0]} and favorite drink is {host_log[1]}")
        if not tts_res.success:
            return 'info_failed'
        
        #for guest more than 1
        if count_guest > 0:
            tts_res = self.text_to_speech_service("and i will present another person in this room, please wait me")
            rospy.loginfo("and i will present another person in this room, please wait me")
            if not tts_res.success:
                return 'info_failed'
            prev_guest = guest_logs[0]
            prev_name = prev_guest[0]
            prev_drink = prev_guest[1]
            prev_pose = prev_guest[2]
            #nav to seat direction
            nav_res = self.nav_service(prev_pose)
            if not nav_res.success:
                rospy.loginfo("move to direction failed")
                return 'info_failed'
            tts_res = self.text_to_speech_service(f"The person sitting here is {prev_name} and favorite drink is {prev_drink}")
            rospy.loginfo(f"The person sitting here is {prev_name} and favorite drink is {prev_drink}")
            if not tts_res.success:
                return 'info_failed'
        
        # curr_guest = guest_logs[count_guest]
        # rospy.loginfo(curr_guest)
        # curr_name = curr_guest[0]
        # curr_drink = curr_guest[1]
        nav_res = self.nav_service("guest_right")
        if not nav_res.success:
            rospy.loginfo("move to direction failed")
            return 'info_failed'
        tts_res = self.text_to_speech_service(str(f"And on my right is {name_of_current_human}, favorite drink is {name_of_current_drink}"))
        rospy.loginfo(f"And on my left is {name_of_current_human}, favorite drink is {name_of_current_drink}")            
        if not tts_res.success:
                return 'info_failed'
        tts_res = self.text_to_speech_service("Nice to meet you all")
        rospy.loginfo("Nice to meet you all")
        if tts_res.success:
            return 'info_provided'
        else:
            return 'info_failed'

#------------------------------------------   wait for ai   -----------------------------------
#this will detect empty seat and robot will turn to empty position
class DetectEmptySeat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detect_seat_done', 'detect_seat_failed'])
        rospy.loginfo('Initializing Detect empty seat state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.empty_seat_detection = '/rip/turtlebot/vision/empty_seat_detection'
        rospy.wait_for_service('/rip/turtlebot/navigation/walk_fix')
        rospy.wait_for_service(self.text_to_speech)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.nav_service = rospy.ServiceProxy('/rip/turtlebot/navigation/walk_fix', NavigationToGoal)
        self.empty_seat_detect_service = rospy.ServiceProxy(self.empty_seat_detection, EmptySeatDetection)
        rospy.loginfo('Detect empty seat state successfully initialized')
    
    def execute(self, ud):
        global count_guest
        global guest_logs
        global name_of_current_human
        global name_of_current_drink

        nav_res = self.nav_service("seat_center")
        if not nav_res.success:
            rospy.loginfo("move to direction failed")
            return 'detect_seat_failed'
        
        tts_res = self.text_to_speech_service("i will find the empty seat please wait")
        rospy.loginfo("i will find the empty seat please wait")            
        if not tts_res.success:
                return 'detect_seat_failed'
        
        empty_seat_res = self.empty_seat_detect_service()
        if empty_seat_res.success:
            seat_empty_pose = empty_seat_res.empty_seat

            #for guest more than 1
            if count_guest > 0:
                prev_guest = guest_logs[0]
                prev_pose = prev_guest[2]
                #for the same seat pose
                if prev_pose == seat_empty_pose:
                    seat_check = ["seat_left", "seat_right", "seat_center"]
                    rospy.loginfo("This seat is not empty")
                    for seat in seat_check:
                        if seat != seat_empty_pose:
                            seat_empty_pose = seat
                #save data
                guest_logs.append([name_of_current_human, name_of_current_drink, seat_empty_pose])

                tts_res = self.text_to_speech_service(f"empty seat is {seat_empty_pose}, please wait me to turn") 
                rospy.loginfo(f"empty seat is {seat_empty_pose}, please wait me to turn") 
                rospy.loginfo(guest_logs)           
                if not tts_res.success:
                    return 'detect_seat_failed'
                #nav to seat direction
                nav_res = self.nav_service(seat_empty_pose)
                if nav_res.success:
                    return 'detect_seat_done'
                return 'detect_seat_failed'
        return 'detect_seat_failed'
#------------------------------------------   wait for ai   -----------------------------------

class CheckGuestOfNumber(smach.State):
    def __init__(self):
        global count_guest
        smach.State.__init__(self, outcomes=['check_all_done', 'check_all_not_done'])
        rospy.loginfo('Initializing Check everyone state...')
        

    def execute(self, ud):
        global guest_logs
        global count_guest
        count_guest = count_guest + 1
        rospy.loginfo("Start Checking Number of Human")
        if count_guest >= max_guest:
            return 'check_all_done'
        else:
            return 'check_all_not_done'

class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'])
        rospy.loginfo('Initializing End state...')

    def execute(self, ud):
        rospy.loginfo('Mission accomplished. Ending state machine.')
        return 'end'

def main():
    rospy.init_node('receptionist_state_machine', anonymous=True)

    sm = smach.StateMachine(outcomes=['mission_completed'])

    with sm:
        rospy.loginfo("state prepare mission")
        smach.StateMachine.add('PREPARE_RECEPTION', PrepareReception(),
                               transitions={'prepare_succeeded': 'READY',
                                            'prepare_failed': 'PREPARE_RECEPTION'})
        
        rospy.loginfo('state 0')
        smach.StateMachine.add('READY', Ready(),
                               transitions={'ready_succeeded': 'NAV_TO_ROOM',
                                            'ready_failed': 'READY'})

        rospy.loginfo('state 1')
        smach.StateMachine.add('NAV_TO_ROOM', NavToRoom(),
                               transitions={'nav_succeeded': 'WAIT_FOR_GUEST',
                                            'nav_failed': 'NAV_TO_ROOM'})
        
        rospy.loginfo("state 2")
        smach.StateMachine.add('WAIT_FOR_GUEST', WaitForGuest(),
                               transitions={'guest_detected': 'GREET_GUEST',
                                            'wait_failed': 'WAIT_FOR_GUEST'})
        
        rospy.loginfo("state 3")
        smach.StateMachine.add('GREET_GUEST', GreetGuest(),
                               transitions={'greet_succeeded': 'ASK_FOR_NAME',
                                            'greet_failed': 'GREET_GUEST'})
        
        rospy.loginfo("state 4")
        smach.StateMachine.add('ASK_FOR_NAME', AskForName(),
                               transitions={'name_received': 'FAVORITE_DRINK_CHECK',
                                            'ask_failed': 'ASK_FOR_NAME'})
        
        rospy.loginfo("state 5")
        smach.StateMachine.add('FAVORITE_DRINK_CHECK', FavoriteDrinkCheck(),
                               transitions={'favorite_drink_received': 'GUIDE_TO_SEAT',
                                            'favorite_drink_failed': 'ASK_FOR_NAME'})
        
        rospy.loginfo("state 6")
        smach.StateMachine.add('GUIDE_TO_SEAT', GuideToSeat(),
                               transitions={'guide_succeeded': 'PROVIDE_INFORMATION',
                                            'guide_failed': 'GUIDE_TO_SEAT'})
        
        rospy.loginfo("state 7")
        smach.StateMachine.add('PROVIDE_INFORMATION', ProvideInformation(),
                               transitions={'info_provided': 'DETECT_EMPTY_SEAT',
                                            'info_failed': 'PROVIDE_INFORMATION'})
        
        rospy.loginfo("state 8")
        smach.StateMachine.add('DETECT_EMPTY_SEAT', DetectEmptySeat(),
                               transitions={'detect_seat_done': 'CHECK_NUMBER_OF_GUEST',
                                            'detect_seat_failed': 'DETECT_EMPTY_SEAT'})
        
        rospy.loginfo('state 9')
        smach.StateMachine.add('CHECK_NUMBER_OF_GUEST', CheckGuestOfNumber(),
                               transitions={'check_all_done': 'END',
                                            'check_all_not_done': 'NAV_TO_ROOM'})
        
        rospy.loginfo('state 10')
        smach.StateMachine.add('END', End(),
                               transitions={'end': 'mission_completed'})

    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
