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
        max_retries = 5
        retry_count = 0

        while retry_count < max_retries:
            text = self.recognize_speech()
            if text == "stop":
                response.text = text
                response.success = True
                self.play_beep_sound()
                
                return response
            else:
                retry_count += 1
                self.call_text_to_speech("Sorry, I couldn't understand. Please try again.")
                self.play_beep_sound()
                self.play_beep_sound()

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

    def recognize_speech(self):
        r = sr.Recognizer()
        with sr.Microphone() as source:
            print("Initializing...")
            r.adjust_for_ambient_noise(source, duration=0.2)
            print("Listening...")
            self.play_beep_sound()
            audio = r.listen(source)
            try:
                text = r.recognize_google(audio).lower()
                print("Recognized text:", text)
                self.play_beep_sound()
                self.play_beep_sound()
                return text
            except sr.UnknownValueError:
                print("Speech recognition could not understand the audio")
            except sr.RequestError as e:
                print(f"Could not request results from Google Speech Recognition service: {e}")
            return None

def main():
    try:
        SpeechToTextNode()
        rospy.spin()
    except rospy.ROSInternalException:
        pass

if __name__ == "__main__":
    main()