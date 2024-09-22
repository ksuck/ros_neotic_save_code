#!/usr/bin/env python3

import rospy
import smach
import smach_ros

from rip_edu_msgs.msg import *

from std_srvs.srv import Trigger, TriggerResponse
from rip_edu_msgs.srv import TextToSpeech, TextToSpeechRequest
from rip_edu_msgs.srv import SpeechToText, SpeechToTextRequest
from rip_edu_msgs.srv import HumanPoseDetection, HumanPoseDetectionRequest
from rip_edu_msgs.srv import NavigationToGoal, NavigationToGoalRequest
from rip_edu_msgs.srv import FreestyleMove, FreestyleMoveRequest
from rip_edu_msgs.srv import MarkHumanPositionToMap, MarkHumanPositionToMapRequest
from rip_edu_msgs.srv import Classification, ClassificationRequest
from rip_edu_msgs.srv import NavToGoalXYT, NavToGoalXYTRequest

#Data
my_mate = {
        "boat": [["Feature1","Feature12","Feature13","Furniture"], [0,0,0]],
        "max":  [["Feature1","Feature12","Feature13","Furniture"], [0,0,0]],
        "girl": [["Feature1","Feature12","Feature13","Furniture"], [0,0,0]]}
name_of_current_human = ""
count_guest = 0
human_position = []

class PrepareMission(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['mission_prepared'])
        rospy.loginfo('Initializing PrepareMission...')
        
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        self.human_pose_detection = '/rip/turtlebot/vision/human_pose_detection'
        self.go_to_location = '/rip/turtlebot/navigation/nav_to_goal_xyt'
        self.freestyle_move = '/rip/turtlebot/navigation/walk_fix'
        self.mark_human_position_to_map = '/navigation/mark_human_position_to_map'
        self.classification_detect = '/rip/turtlebot/vision/classification_detect'

        # print(my_mate)

        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.human_pose_detection)
        rospy.wait_for_service(self.go_to_location)
        rospy.wait_for_service(self.mark_human_position_to_map)
        rospy.wait_for_service(self.classification_detect)

        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        self.human_pose_detection_service = rospy.ServiceProxy(self.human_pose_detection, HumanPoseDetection)
        self.go_to_location_service = rospy.ServiceProxy(self.go_to_location, NavToGoalXYT)
        self.freestyle_move_service = rospy.ServiceProxy(self.freestyle_move, FreestyleMove)
        self.mark_human_position_to_map_service = rospy.ServiceProxy(self.mark_human_position_to_map, MarkHumanPositionToMap)
        self.classification_detect_service = rospy.ServiceProxy(self.classification_detect, Classification)
        rospy.loginfo('PrepareMission successfully initialized')

    def execute(self, ud):
        return 'mission_prepared'

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

    def execute(self, ud):
        rospy.wait_for_service('/rip/turtlebot/navigation/walk_fix')
        nav_service = rospy.ServiceProxy('/rip/turtlebot/navigation/walk_fix', NavigationToGoal)
        nav_res = nav_service("livingroom")
        if nav_res.success:
            return 'nav_succeeded'
        else:
            return 'nav_failed'
        
# ----------------------------------------------------------------------------------------------------------------------     !!!!!!!!!!     wait for navigation to explanning how to save position
class HumanDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detection_succeeded', 'detection_failed'])
        rospy.loginfo('Initializing HumanDetection state...')

    def execute(self, ud):
        rospy.wait_for_service('/rip/turtlebot/vision/human_pose_detection')
        human_detection_service = rospy.ServiceProxy('/rip/turtlebot/vision/human_pose_detection', HumanPoseDetection)
        rospy.loginfo("Start Detect All Human")
        detection_res = human_detection_service()
        if detection_res.success:
            # Save human positions to userdata
            ud.human_positions = detection_res.human_position
            return 'detection_succeeded'
        else:
            return 'detection_failed'

class MarkPoseToMap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['mark_succeeded', 'mark_failed'],
                             input_keys=['human_positions'],
                             output_keys=['marked_positions'])
        rospy.loginfo('Initializing MarkPoseToMap state...')

    def execute(self, ud):
        rospy.wait_for_service('/navigation/mark_human_position_to_map')
        mark_service = rospy.ServiceProxy('/navigation/mark_human_position_to_map', MarkHumanPositionToMap)
        rospy.loginfo("Start Marking Human Position In To Map")
        mark_res = mark_service(ud.human_positions)
        if mark_res.success:
            ud.marked_positions = mark_res.human_position_in_map
            return 'mark_succeeded'
        else:
            return 'mark_failed'
# ----------------------------------------------------------------------------------------------------------------------

class NavToNearestHumanPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_succeeded', 'nav_failed'],
                             input_keys=['marked_positions'])
        rospy.loginfo('Initializing NavToNearestHumanPose state...')

    def execute(self, ud):
        rospy.loginfo(count_guest)
        rospy.wait_for_service(self.go_to_location)
        self.go_to_location_service = rospy.ServiceProxy(self.go_to_location, NavToGoalXYT)
        rospy.loginfo("Start Navigation To Nearest Position")
        human_position = ud.marked_positions[count_guest]
        x = human_position[0]
        y = human_position[1]
        theta = human_position[2]
        nav_res = self.go_to_location_service(x, y, theta)
        if nav_res.success:
            return 'nav_succeeded'
        return 'nav_failed'

class TTSAskGuestName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ask_succeeded', 'ask_failed'])
        rospy.loginfo('Initializing TTSAskGuestName state...')

    def execute(self, ud):
        rospy.loginfo("Start Check Human In Database")
        rospy.wait_for_service('/rip/turtlebot/voice/text_to_speech')
        text_to_speech_service = rospy.ServiceProxy('/rip/turtlebot/voice/text_to_speech', TextToSpeech)
        rospy.loginfo('TextToSpeech service is ready')

        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        rospy.wait_for_service(self.speech_to_text)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        rospy.loginfo('SpeechToText service is ready')

        for human_name in my_mate:
            tts_res = text_to_speech_service("Are you " + human_name)
            if tts_res.success:
                stt_res = self.speech_to_text_service()
                name_check = "y" in stt_res or "Y" in stt_res
                if name_check:
                    name_of_current_human = human_name
                    rospy.loginfo("This is his name.")
                    # my_mate[name_of_current_human][1] = list_position #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!     wait for navigation to get position
                    return 'ask_succeeded'
                else:
                    rospy.loginfo("This isn't his name, try to check another name")
            else:
                return 'ask_failed'
        if name_of_current_human == "":
            return 'ask_failed'

class TTSGreetingGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['greet_succeeded', 'greet_failed'])
        rospy.loginfo('Initializing TTSGreetingGuest state...')

    def execute(self, ud):
        rospy.wait_for_service('/rip/turtlebot/voice/text_to_speech')
        text_to_speech_service = rospy.ServiceProxy('/rip/turtlebot/voice/text_to_speech', TextToSpeech)
        tts_res = text_to_speech_service("Hello, welcome. Please stand here and i will take your picture. ")
        if tts_res.success:
            return 'greet_succeeded'
        else:
            return 'greet_failed'

class DetectAllFeatures(smach.State): # process time not sure.
    def __init__(self):
        smach.State.__init__(self, outcomes=['detection_succeeded', 'detection_failed'])
        rospy.loginfo('Initializing DetectAllFeatures state...')

    def execute(self, ud):
        rospy.loginfo("Start Detect all features of human")
        rospy.wait_for_service('/rip/turtlebot/vision/classification_detect')
        detect_service = rospy.ServiceProxy('/rip/turtlebot/vision/classification_detect', Classification)
        detect_res = detect_service()
        if detect_res.success:
            # Save detected features to userdata
            detected_features = detect_res.list_of_class
            rospy.loginfo(detected_features)
            my_mate[name_of_current_human][0] = detected_features
            my_mate[name_of_current_human][1] = human_position
            return 'detection_succeeded'
        else:
            return 'detection_failed'

class NavToOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_succeeded', 'nav_failed'])
        rospy.loginfo('Initializing NavToOperator state...')

    def execute(self, ud):
        rospy.loginfo("Start Navigation to operator")
        rospy.wait_for_service('/rip/turtlebot/navigation/walk_fix')
        nav_service = rospy.ServiceProxy('/rip/turtlebot/navigation/walk_fix', NavigationToGoal)
        nav_res = nav_service("operator")
        if nav_res.success:
            return 'nav_succeeded'
        else:
            return 'nav_failed'

class TTSTellDetectedFeatures(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tell_succeeded', 'tell_failed'],
                             input_keys=['detected_features'])
        rospy.loginfo('Initializing TTSTellDetectedFeatures state...')

    def execute(self, ud):
        rospy.loginfo("Start Tell all features to operator")
        rospy.wait_for_service('/rip/turtlebot/voice/text_to_speech')
        text_to_speech_service = rospy.ServiceProxy('/rip/turtlebot/voice/text_to_speech', TextToSpeech)
        # features_str = ", ".join(ud.detected_features)
        # tts_res = text_to_speech_service("I have detected the following features: " + features_str)

        tts_res = text_to_speech_service( name_of_current_human + 
                                         "wears " + my_mate[name_of_current_human][0][0] + 
                                         ", a " + my_mate[name_of_current_human][0][1] + 
                                         ", a " + my_mate[name_of_current_human][0][2] + 
                                         " and " + name_of_current_human + 
                                         " near by" + my_mate[name_of_current_human][0][3])
        if tts_res.success:
            return 'tell_succeeded'
        else:
            return 'tell_failed'
        

class CheckGuestOfNumber(smach.state):
    def __init__(self):
        smach.State.__init__(self, outcomes=['check_all_done', 'check_all_not_done'])
        rospy.loginfo('Initializing Check everyone state...')
        count_guest = count_guest + 1

    def execute(self, ud):
        rospy.loginfo("Start Checking Number of Human")
        if count_guest == max(my_mate):
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
    rospy.init_node('find_my_mate_mission')

    sm = smach.StateMachine(outcomes=['mission_completed'])

    with sm:
        smach.StateMachine.add('PREPARE_MISSION', PrepareMission(), 
                               transitions={'mission_prepared': 'READY'})

        rospy.loginfo('1')
        smach.StateMachine.add('READY', Ready(),
                               transitions={'ready_succeeded': 'NAV_TO_ROOM',
                                            'ready_failed': 'READY'})

        rospy.loginfo('2')
        smach.StateMachine.add('NAV_TO_ROOM', NavToRoom(),
                               transitions={'nav_succeeded': 'HUMAN_DETECTION',
                                            'nav_failed': 'NAV_TO_ROOM'})

        rospy.loginfo('3')
        smach.StateMachine.add('HUMAN_DETECTION', HumanDetection(),
                               transitions={'detection_succeeded': 'MARK_POSE_TO_MAP',
                                            'detection_failed': 'HUMAN_DETECTION'})

        rospy.loginfo('4')
        smach.StateMachine.add('MARK_POSE_TO_MAP', MarkPoseToMap(),
                               transitions={'mark_succeeded': 'NAV_TO_NEAREST_HUMAN_POSE',
                                            'mark_failed': 'MARK_POSE_TO_MAP'})

        rospy.loginfo('5')
        smach.StateMachine.add('NAV_TO_NEAREST_HUMAN_POSE', NavToNearestHumanPose(),
                               transitions={'nav_succeeded': 'TTS_ASK_GUEST_NAME',
                                            'nav_failed': 'NAV_TO_NEAREST_HUMAN_POSE'})

        rospy.loginfo('6')
        smach.StateMachine.add('TTS_ASK_GUEST_NAME', TTSAskGuestName(),
                               transitions={'ask_succeeded': 'TTS_GREETING_GUEST',
                                            'ask_failed': 'TTS_ASK_GUEST_NAME'})

        rospy.loginfo('7')
        smach.StateMachine.add('TTS_GREETING_GUEST', TTSGreetingGuest(),
                               transitions={'greet_succeeded': 'DETECT_ALL_FEATURES',
                                            'greet_failed': 'TTS_GREETING_GUEST'})

        rospy.loginfo('8')
        smach.StateMachine.add('DETECT_ALL_FEATURES', DetectAllFeatures(),
                               transitions={'detection_succeeded': 'NAV_TO_OPERATOR',
                                            'detection_failed': 'DETECT_ALL_FEATURES'})
        
        rospy.loginfo('9')
        smach.StateMachine.add('NAV_TO_OPERATOR', NavToOperator(),
                               transitions={'nav_succeeded': 'TTS_TELL_DETECTED_FEATURES',
                                            'nav_failed': 'NAV_TO_OPERATOR'})

        rospy.loginfo('10')
        smach.StateMachine.add('TTS_TELL_DETECTED_FEATURES', TTSTellDetectedFeatures(),
                               transitions={'tell_succeeded': 'CHECK_NUMBER_OF_GUEST',
                                            'tell_failed': 'TTS_TELL_DETECTED_FEATURES'})
        
        rospy.loginfo('11')
        smach.StateMachine.add('CHECK_NUMBER_OF_GUEST', CheckGuestOfNumber(),
                               transitions={'check_all_done': 'END',
                                            'check_all_not_done': 'NAV_TO_NEAREST_HUMAN_POSE'})

        rospy.loginfo('12')
        smach.StateMachine.add('END', End(),
                               transitions={'end': 'mission_completed'})

    sis = smach_ros.IntrospectionServer('find_my_mate_mission', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
