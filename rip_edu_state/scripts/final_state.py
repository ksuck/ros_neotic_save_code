#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_srvs.srv import Empty, Trigger
from rip_edu_msgs.srv import *
from rip_edu_msgs.msg import *
from rospkg import RosPack
import os
import time
global guest_logs
global count_room
global list_of_rooms
pack_path = RosPack().get_path('turtlebot_follower')

class PrepareReception(smach.State):
    def __init__(self):
        global guest_logs
        global count_room
        
        global list_of_rooms
        
        
        list_of_rooms = ["kitchen", "livingroom", "study", "bedroom"]
        guest_logs = []
        count_room = 0

        smach.State.__init__(self, outcomes=['prepare_succeeded', 'prepare_failed'])
        rospy.loginfo('Initializing PrepareReception...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        self.detect_human_in_front = '/rip/turtlebot/vision/detect_human_in_front'
        self.empty_seat_detection = '/rip/turtlebot/vision/empty_seat_detection'
        self.nav_to_goal = '/rip/turtlebot/navigation/walk_fix'
        
        rospy.loginfo('text_to_speech service is ready')
        rospy.wait_for_service(self.text_to_speech)
        rospy.loginfo('speech_to_text service is ready')
        rospy.wait_for_service(self.speech_to_text)
        rospy.loginfo('detect_human_in_front service is ready')
        rospy.wait_for_service(self.detect_human_in_front)
        rospy.loginfo('empty_seat_detection service is ready')
        rospy.wait_for_service(self.empty_seat_detection)
        rospy.loginfo('nav_to_goal service is ready')
        rospy.wait_for_service(self.nav_to_goal)

        self.run_launch_file = '/rip/turtlebot/state/run_launch_file'
        rospy.wait_for_service(self.run_launch_file)
        rospy.loginfo('RunLaunchFile service is ready')
        self.stop_follower = '/rip/turtlebot/state/stop_follower'
        rospy.wait_for_service(self.stop_follower)
        rospy.loginfo('StopFollower service is ready')

        self.move_to_position = '/rip/turtlebot/manipulator/move_to_position'
        rospy.wait_for_service(self.move_to_position)

        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        self.detect_human_in_front_service = rospy.ServiceProxy(self.detect_human_in_front, DetectHumanInFront)
        self.empty_seat_detect_service = rospy.ServiceProxy(self.empty_seat_detection, EmptySeatDetection)
        self.nav_to_goal_service = rospy.ServiceProxy(self.nav_to_goal, NavigationToGoal)
        self.run_launch_file_service = rospy.ServiceProxy(self.run_launch_file, RunLaunch)
        self.stop_follower_service = rospy.ServiceProxy(self.stop_follower, Trigger)
        self.move_to_position_service = rospy.ServiceProxy(self.move_to_position, RipMoveGroup)
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
        tts_res = text_to_speech_service("I'm ready for the final task")
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
        global count_room
        global list_of_rooms

        nav_res = self.nav_to_goal_service(list_of_rooms[count_room])
        if nav_res.success:
            return 'nav_succeeded'
        else:
            return 'nav_failed'

class DetectInjuredHuman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['injured_detected', 'injured_detect_failed', 'no_injured_human'])
        rospy.loginfo('Initializing Detect Detect injured human state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.empty_seat_detection = '/rip/turtlebot/vision/empty_seat_detection'
        rospy.wait_for_service('/rip/turtlebot/navigation/walk_fix')
        rospy.wait_for_service(self.text_to_speech)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.nav_service = rospy.ServiceProxy('/rip/turtlebot/navigation/walk_fix', NavigationToGoal)
        self.empty_seat_detect_service = rospy.ServiceProxy(self.empty_seat_detection, EmptySeatDetection)
        rospy.loginfo('Detect empty seat state successfully initialized')
    
    def execute(self, ud):
        global count_room
        global list_of_rooms

        # retry_loop = 2
        # for 
        # if count_room == 3:
            
        #     return 'injured_detected'
        # else:
        #     count_room = count_room + 1
        #     return 'no_injured_human'
        
        empty_seat_res = self.empty_seat_detect_service()
        rospy.loginfo(f"the ans is {empty_seat_res.empty_seat}")
        if empty_seat_res.success:
            if empty_seat_res.empty_seat == 'injured':
                tts_res = self.text_to_speech_service("i see you lay on the ground. Are you ok?")
                rospy.loginfo("i see you lay on the ground. Are you ok?")         
                if not tts_res.success:
                    return 'injured_detect_failed'
                for i in range(3):
                    tts_res = self.text_to_speech_service("Are you okay?")
                    rospy.loginfo("Are you okay?")         
                    if not tts_res.success:
                        return 'injured_detect_failed'
                    time.sleep(1)
                empty_seat_res = self.empty_seat_detect_service()
        if empty_seat_res.success:
            if empty_seat_res.empty_seat == 'injured':
                tts_res = self.text_to_speech_service(f"Some one in {list_of_rooms[count_room]} room injured, i will take someone to help you")
                rospy.loginfo("Some one in this room injured, i will take someone to help you")         
                if not tts_res.success:
                    return 'injured_detect_failed'
                return 'injured_detected'
            rospy.loginfo("Everythong is ok")
            count_room = count_room + 1
            if count_room >= 4:
                count_room = 0
            return 'no_injured_human'
        return 'injured_detect_failed'
    
class NavToCenter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_center_succeeded', 'nav_center_failed'])
        rospy.loginfo('Initializing NavToRoom state...')
        self.nav_to_goal = '/rip/turtlebot/navigation/walk_fix'
        rospy.wait_for_service(self.nav_to_goal)
        self.nav_to_goal_service = rospy.ServiceProxy(self.nav_to_goal, NavigationToGoal)

    def execute(self, ud):
        global list_of_rooms

        self.play_beep_sound()
        nav_res = self.nav_to_goal_service('outroom')
        if nav_res.success:
            return 'nav_center_succeeded'
        else:
            return 'nav_center_failed'
        
    def play_beep_sound(self):
        os.system('play -n synth 0.2 sin 800')
        
class GreetGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['greet_succeeded', 'greet_failed'])
        rospy.loginfo('Initializing GreetGuest state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.loginfo('GreetGuest state successfully initialized')

    def execute(self, ud):
        self.detect_human_in_front = '/rip/turtlebot/vision/detect_human_in_front'
        rospy.wait_for_service(self.detect_human_in_front)
        self.detect_human_in_front_service = rospy.ServiceProxy(self.detect_human_in_front, DetectHumanInFront)

        rospy.loginfo('Greeting the guest...')
        tts_res = self.text_to_speech_service("Someone in this room get injured please help")
        if not tts_res.success:
            return 'greet_failed'

        res = self.detect_human_in_front_service()
        if res.success:
            rospy.loginfo('Guest detected')
            tts_res = self.text_to_speech_service("Please follow me, i wiil take you there")
            if not tts_res.success:
                return 'greet_failed'
            return 'greet_succeeded'
        return 'greet_failed'
    
class NavToInjuredRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_injured_room_succeeded', 'nav_injured_room_failed'])
        rospy.loginfo('Initializing NavToRoom state...')
        self.nav_to_goal = '/rip/turtlebot/navigation/walk_fix'
        rospy.wait_for_service(self.nav_to_goal)
        self.nav_to_goal_service = rospy.ServiceProxy(self.nav_to_goal, NavigationToGoal)
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)

    def execute(self, ud):
        global count_room
        global list_of_rooms

        nav_res = self.nav_to_goal_service(list_of_rooms[count_room])
        if nav_res.success:
            count_room = 0
            tts_res = self.text_to_speech_service("We have arrived")
            if not tts_res.success:
                return 'nav_injured_room_failed'
            return 'nav_injured_room_succeeded'
        else:
            return 'nav_injured_room_failed'
        
class ProvideInformation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['info_provided', 'info_failed'])
        rospy.loginfo('Initializing ProvideInformation state...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.wait_for_service('/rip/turtlebot/navigation/walk_fix')
        self.nav_service = rospy.ServiceProxy('/rip/turtlebot/navigation/walk_fix', NavigationToGoal)
        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        rospy.wait_for_service(self.speech_to_text)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        rospy.loginfo('ProvideInformation state successfully initialized')

    def execute(self, ud):
        rospy.loginfo('Providing information to the guest...')
        tts_res = self.text_to_speech_service('Do you need me to call the emergency number 112')
        rospy.loginfo('Do you need me to call the emergency number 112')
        if not tts_res.success:
            return 'info_failed'
        stt_res = self.speech_to_text_service()
        rospy.loginfo(stt_res)
        if stt_res.success:
            name_check = ("ye" in stt_res.text or "Ye" in stt_res.text)
            if name_check:
                tts_res = self.text_to_speech_service('i am calling')
                rospy.loginfo('i am calling')
                if not tts_res.success:
                    return 'info_failed'
                return 'info_provided'
            return 'info_provided'
        return 'info_failed'

class StartFollower(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_follower_succeeded', 'start_follower_failed'])
        rospy.loginfo('Initialize StartFollower state in progress...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        rospy.loginfo('TextToSpeech service is ready')
        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        rospy.wait_for_service(self.speech_to_text)
        rospy.loginfo('SpeechToText service is ready')
        self.run_launch_file = '/rip/turtlebot/state/run_launch_file'
        rospy.wait_for_service(self.run_launch_file)
        rospy.loginfo('RunLaunchFile service is ready')
        self.stop_follower = '/rip/turtlebot/state/stop_follower'
        rospy.wait_for_service(self.stop_follower)
        rospy.loginfo('StopFollower service is ready')
        self.move_to_position = '/rip/turtlebot/manipulator/move_to_position'
        rospy.wait_for_service(self.move_to_position)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        self.run_launch_file_service = rospy.ServiceProxy(self.run_launch_file, RunLaunch)
        self.stop_follower_service = rospy.ServiceProxy(self.stop_follower, Trigger)
        self.move_to_position_service = rospy.ServiceProxy(self.move_to_position, RipMoveGroup)
        rospy.loginfo('Initialize StartFollower state success')

    def execute(self, ud):
        tts_res = self.text_to_speech_service("Do you want me to carry the something?")
        if not tts_res.success:
            return 'start_follower_failed'
        stt_res = self.speech_to_text_service()
        rospy.loginfo(stt_res.text)
        if ("ye" in stt_res.text or "Ye" in stt_res.text):
            arm_res = self.move_to_position_service([90, 20, 40], 'FK', 1)
            if not arm_res.success:
                return 'start_follower_failed'
            launch_res = self.run_launch_file_service(pack_path + '/launch/follower.launch')

            if not launch_res.success:
                return 'start_follower_failed'

            tts_res = self.text_to_speech_service("I'm starting to follow you, Say stop to stop the follower.")
            if not tts_res.success:
                return 'start_follower_failed'

            res_stt = self.speech_to_text_service()
            rospy.loginfo(res_stt.text)
            # split_text = res_stt.text.split()
            # while (split_text.count("stop")) == 1:
            while ('stop' in res_stt.text ):
                # stop_res = self.stop_follower_service(pack_path + '/launch/follower.launch')
                stop_res = self.stop_follower_service()
                if stop_res.success:
                    tts_res = self.text_to_speech_service("I stopped following you. please take the your thing.")
                    if tts_res.success:
                        return 'start_follower_succeeded'
                    else:
                        return 'start_follower_failed'
                else:
                    return 'start_follower_failed'
        else:
            return 'start_follower_succeeded'
        
class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'])
        rospy.loginfo('Initializing End state...')

    def execute(self, ud):
        rospy.loginfo('Mission accomplished. Ending state machine.')
        rospy.wait_for_service('/rip/turtlebot/voice/text_to_speech')
        text_to_speech_service = rospy.ServiceProxy('/rip/turtlebot/voice/text_to_speech', TextToSpeech)
        tts_res = text_to_speech_service("Good luck!, I finished the task")
        if tts_res.success:
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
        # rospy.loginfo('state 0')
        # smach.StateMachine.add('READY', Ready(),
        #                        transitions={'ready_succeeded': 'DETECT_INJURED_HUMAN',
        #                                     'ready_failed': 'READY'})

        rospy.loginfo('state 1')
        smach.StateMachine.add('NAV_TO_ROOM', NavToRoom(),
                               transitions={'nav_succeeded': 'DETECT_INJURED_HUMAN',
                                            'nav_failed': 'NAV_TO_ROOM'})
        
        rospy.loginfo("state 2")
        smach.StateMachine.add('DETECT_INJURED_HUMAN', DetectInjuredHuman(),
                               transitions={'injured_detected': 'NAV_TO_CENTER',
                                            'injured_detect_failed': 'DETECT_INJURED_HUMAN',
                                            'no_injured_human' : 'NAV_TO_ROOM'})
        
        rospy.loginfo('state 3')
        smach.StateMachine.add('NAV_TO_CENTER', NavToCenter(),
                               transitions={'nav_center_succeeded': 'GREET_GUEST',
                                            'nav_center_failed': 'NAV_TO_CENTER'})
        
        rospy.loginfo("state 4")
        smach.StateMachine.add('GREET_GUEST', GreetGuest(),
                               transitions={'greet_succeeded': 'NAV_TO_INJURED_ROOM',
                                            'greet_failed': 'GREET_GUEST'})
        
        rospy.loginfo('state 5')
        smach.StateMachine.add('NAV_TO_INJURED_ROOM', NavToInjuredRoom(),
                               transitions={'nav_injured_room_succeeded': 'PROVIDE_INFORMATION',
                                            'nav_injured_room_failed': 'NAV_TO_INJURED_ROOM'})

        rospy.loginfo("state 6")
        smach.StateMachine.add('PROVIDE_INFORMATION', ProvideInformation(),
                               transitions={'info_provided': 'START_FOLLOWER',
                                            'info_failed': 'PROVIDE_INFORMATION'})
        
        rospy.loginfo('state 7')
        smach.StateMachine.add('START_FOLLOWER', StartFollower(),
                                 transitions={'start_follower_succeeded': 'END',
                                              'start_follower_failed': 'START_FOLLOWER'})
        
        rospy.loginfo('state 8')
        smach.StateMachine.add('END', End(),
                               transitions={'end': 'mission_completed'})

    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
