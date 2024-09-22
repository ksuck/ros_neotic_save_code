#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from rip_edu_msgs.srv import *
from rip_edu_msgs.msg import *
from std_srvs.srv import Trigger , Empty
from rospkg import RosPack

hand_direction = ""
pack_path = RosPack().get_path('turtlebot_follower')

class PrepareMission(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[])
        rospy.loginfo('Initializing PrepareMission...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        self.speech_to_text = '/rip/turtlebot/voice/speech_to_text'
        self.hand_detection = '/rip/turtlebot/vision/hand_detection'
        self.freestyle_move = '/rip/turtlebot/navigation/walk_fix'
        self.move_to_position = '/rip/turtlebot/manipulator/move_to_position'
        self.run_launch_file = '/rip/turtlebot/state/run_launch_file'
        self.stop_follower = '/rip/turtlebot/state/stop_follower'

        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.speech_to_text)
        rospy.wait_for_service(self.hand_detection)
        rospy.wait_for_service(self.freestyle_move)
        rospy.wait_for_service(self.move_to_position)
        rospy.wait_for_service(self.run_launch_file)
        rospy.wait_for_service(self.stop_follower)

        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, Empty)
        self.hand_detection_service = rospy.ServiceProxy(self.hand_detection, HandDetection)
        self.freestyle_move_service = rospy.ServiceProxy(self.freestyle_move, FreestyleMove)
        self.move_to_position_service = rospy.ServiceProxy(self.move_to_position, RipMoveGroup)
        self.run_launch_file_service = rospy.ServiceProxy(self.run_launch_file, RunLaunch)
        self.stop_follower_service = rospy.ServiceProxy(self.stop_follower, Trigger)
        rospy.loginfo('PrepareMission successfully')

class Ready(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready_succeeded', 'ready_failed'])
        rospy.loginfo('Initialize Ready state in progress...')
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        rospy.loginfo('TextToSpeech service is ready')
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.loginfo('Initialize Ready state success')

    def execute(self, ud):
        res = self.text_to_speech_service("I'm ready to do carry my luggage task")

        
        if res.success:
            rospy.loginfo('Ready succeeded')
            return 'ready_succeeded'
        else:
            return 'ready_failed'

class DetectHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detect_hand_succeeded', 'detect_hand_failed'])
        rospy.loginfo('Initialize DetectHand state in progress...')
        self.hand_detection = '/rip/turtlebot/vision/hand_detection'
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.hand_detection)
        rospy.loginfo('HandDetection service is ready')
        rospy.loginfo('TextToSpeech service is ready')
        self.hand_detection_service = rospy.ServiceProxy(self.hand_detection, HandDetection)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
  
        
        rospy.loginfo('Initialize DetectHand state success')

    def execute(self, ud):
        global hand_direction

        rospy.loginfo('HandDetection state is executing...')
        tts_res = self.text_to_speech_service("Please point to the bag that you want me to carry")

        if not tts_res.success:
            return 'detect_hand_failed'

        timeout = 10.0
        start_time = rospy.get_time()


        rospy.loginfo('Hand detection start loop ')
        while (rospy.get_time() - start_time) < timeout:
            hand_res = self.hand_detection_service()
            detected_hand = hand_res.success
            detected_direction = hand_res.direction

            if detected_hand:
                hand_direction = detected_direction
                tts_res = self.text_to_speech_service("I see your hand pointing to the " + detected_direction)
                if not tts_res.success:
                    return 'detect_hand_failed'
                return 'detect_hand_succeeded'
        rospy.loginfo('Hand detection timeout after ' + str(timeout) + ' seconds.')
        
        self.retry_count += 1
        if self.retry_count < 3:
            tts_res = self.text_to_speech_service("I didn't detect your hand. Let's try again.")
            if not tts_res.success:
                return 'detect_hand_failed'
            return 'detect_hand_failed'
        else:
            tts_res = self.text_to_speech_service("I couldn't detect your hand after multiple attempts. Aborting the task.")
            if not tts_res.success:
                return 'detect_hand_failed'
            return 'detect_hand_failed'

class MoveToBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_to_bag_succeeded', 'move_to_bag_failed'])
        rospy.loginfo("initialize MoveToBag state in progress...")
        self.freestyle_move = '/rip/turtlebot/navigation/walk_fix'
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.freestyle_move)
        rospy.loginfo("TextToSpeech service is ready")
        rospy.loginfo("FreestyleMove service is ready")
        self.freestyle_move_service = rospy.ServiceProxy(self.freestyle_move, FreestyleMove)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.loginfo("initialize MoveToBag state success")

    def execute(self, ud):
        tts_res = self.text_to_speech_service("Moving to the bag.")

        if not tts_res.success:
            return 'move_to_bag_failed'

        res = self.freestyle_move_service(hand_direction)
        if res.success:
            return 'move_to_bag_succeeded'
        else:
            return 'move_to_bag_failed'

class CarryBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['carry_bag_succeeded', 'carry_bag_failed'])
        rospy.loginfo('Initialize CarryBag state in progress')
        self.move_to_position = '/rip/turtlebot/manipulator/move_to_position'
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.move_to_position)
        rospy.wait_for_service(self.text_to_speech)
        self.move_to_position_service = rospy.ServiceProxy(self.move_to_position, RipMoveGroup)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.loginfo('MoveToPosition service is ready')

    def execute(self, ud):
        # self.move_to_position_service([90, 0, 0], 'FK', 0)
        tts_res = self.text_to_speech_service("I will open my hand then please give me the bag")
        if not tts_res.success:
            return 'carry_bag_failed'
        
        self.move_to_position_service([90, 80, 105], 'FK', 1)
        self.move_to_position_service([90, 120, 160], 'FK', 1)
        self.move_to_position_service([90, 120, 160], 'FK', 0)
        self.move_to_position_service([90, 80, 120], 'FK', 0)
        res = self.move_to_position_service([90, 20, 40], 'FK', 0)

        if res.success:
            return 'carry_bag_succeeded'

        return 'carry_bag_failed'

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
        rospy.loginfo('MoveToPosition service is ready')
        self.move_to_position_service = rospy.ServiceProxy(self.move_to_position, RipMoveGroup)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        self.speech_to_text_service = rospy.ServiceProxy(self.speech_to_text, SpeechToText)
        self.run_launch_file_service = rospy.ServiceProxy(self.run_launch_file, RunLaunch)
        self.stop_follower_service = rospy.ServiceProxy(self.stop_follower, Trigger)
        rospy.loginfo('Initialize StartFollower state success')

    def execute(self, ud):
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
                tts_res = self.text_to_speech_service("I stopped following you. please take the bag.")
                if tts_res.success:
                    self.move_to_position_service([90, 100, 60], 'FK', 0)
                    self.move_to_position_service([90, 100, 60], 'FK', 1)
                    arm_res = self.move_to_position_service([90, 0, 0], 'FK', 0)
                    if arm_res.success:
                        return 'start_follower_succeeded'
                    else:
                        return 'start_follower_failed'
                else:
                    return 'start_follower_failed'
            else:
                return 'start_follower_failed'

class MoveToStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_to_start_succeeded', 'move_to_start_failed'])
        rospy.loginfo("initialize MoveToStart state in progress...")
        self.freestyle_move = '/rip/turtlebot/navigation/walk_fix'
        self.text_to_speech = '/rip/turtlebot/voice/text_to_speech'
        rospy.wait_for_service(self.text_to_speech)
        rospy.wait_for_service(self.freestyle_move)
        rospy.loginfo("TextToSpeech service is ready")
        rospy.loginfo("WalkFix service is ready")
        self.walk_fix_service = rospy.ServiceProxy(self.freestyle_move, FreestyleMove)
        self.text_to_speech_service = rospy.ServiceProxy(self.text_to_speech, TextToSpeech)
        rospy.loginfo("initialize MoveToStart state success")

    def execute(self, ud):
        tts_res = self.text_to_speech_service("I'm moving to the starting point.")

        if not tts_res.success:
            return 'move_to_start_failed'

        res = self.walk_fix_service('start')
        if res.success:
            tts_res = self.text_to_speech_service("I'm done with the Carry My Luggage Task.")

            if not tts_res.success:
                return 'move_to_start_failed'

            return 'move_to_start_succeeded'
        else:
            return 'move_to_start_failed'


def main():
    rospy.init_node('cml_state', anonymous=True)

    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add('READY', Ready(),
                               transitions={'ready_succeeded': 'DETECT_HAND',
                                            'ready_failed': 'READY'})
        
        # rospy.loginfo('1')
        # smach.StateMachine.add('DETECT_HAND', DetectHand(),
        #                        transitions={'detect_hand_succeeded': 'MOVE_TO_BAG',
        #                                     'detect_hand_failed': 'DETECT_HAND'})
        rospy.loginfo('1')
        smach.StateMachine.add('DETECT_HAND', DetectHand(),
                               transitions={'detect_hand_succeeded': 'CARRY_BAG',
                                            'detect_hand_failed': 'DETECT_HAND'})
        

        rospy.loginfo('2')
        smach.StateMachine.add('MOVE_TO_BAG', MoveToBag(),
                                 transitions={'move_to_bag_succeeded': 'CARRY_BAG',
                                              'move_to_bag_failed': 'MOVE_TO_BAG'})
        
        rospy.loginfo('3')
        smach.StateMachine.add('CARRY_BAG', CarryBag(),
                               transitions={'carry_bag_succeeded': 'START_FOLLOWER',
                                            'carry_bag_failed': 'CARRY_BAG'})
        
        rospy.loginfo('4')
        smach.StateMachine.add('START_FOLLOWER', StartFollower(),
                                 transitions={'start_follower_succeeded': 'MOVE_TO_START',
                                              'start_follower_failed': 'START_FOLLOWER'})
        
        rospy.loginfo('5')
        smach.StateMachine.add('MOVE_TO_START', MoveToStart(),
                                    transitions={'move_to_start_succeeded': 'done',
                                                'move_to_start_failed': 'MOVE_TO_START'})

    sis = smach_ros.IntrospectionServer('cml', sm, '/SM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()