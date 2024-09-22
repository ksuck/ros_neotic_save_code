#!/usr/bin/env python3

import rospy
import speech_recognition as sr
from rip_edu_msgs.srv import SpeechToText , SpeechToTextResponse
from rip_edu_msgs.srv import TextToSpeech
from std_srvs.srv import Empty
import os

class SpeechToTextNode:
    def __init__(self):
        self.init = rospy.init_node("speech_to_text", anonymous=True)
        self.speech_to_text = rospy.Service("/rip/turtlebot/voice/speech_to_text", SpeechToText, self.callback)
        rospy.loginfo('SpeechToText service is ready')

    def callback(self, req):
        response = SpeechToTextResponse()
        r = sr.Recognizer()
        text = ''
        print("Listening...")
        # self.call_text_to_speech("Listening and please speak loudly")
        while (text == ''):
            with sr.Microphone() as source:
                print("Initializing...")
                r.adjust_for_ambient_noise(source, duration=0.08)
                audio = r.listen(source)
                try:
                    text = r.recognize_google(audio).lower()
                    print("Recognized text:", text)
                    # self.call_text_to_speech("I hear you say",text)
                    
                    if text != '':
                        response.text = text
                        # self.call_text_to_speech(f"I hear you say {text}")
                        response.success = True
                        self.play_beep_sound()
                        # self.call_text_to_speech("The robot will stop following")
                        return response
                except sr.UnknownValueError:
                    self.call_text_to_speech("I don't know what you say and please speak loudly")
                    print("Speech recognition could not understand the audio")
                except sr.RequestError as e:
                    print(f"Could not request results from Google Speech Recognition service: {e}")
                    break
        self.call_text_to_speech("I don't know what you say and please speak loudly")
        response.text = ""
        response.success = False
        return response

    def call_text_to_speech(self, text):
        rospy.wait_for_service('/rip/turtlebot/voice/text_to_speech')
        try:
            text_to_speech_service = rospy.ServiceProxy('/rip/turtlebot/voice/text_to_speech', TextToSpeech)
            response = text_to_speech_service(text)
            return response.success
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return False

    def play_beep_sound(self):
        os.system('play -n synth 0.2 sin 800')

def main():
    try:
        SpeechToTextNode()
        rospy.spin()
    except rospy.ROSInternalException:
        pass

if __name__ == "__main__":
    main()