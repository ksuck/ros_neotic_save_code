#!/usr/bin/env python3

import rospy
from gtts import gTTS
from rip_edu_msgs.srv import TextToSpeech, TextToSpeechResponse
import os
import tempfile

class TextToSpeechNode:
    def __init__(self):
        self.init = rospy.init_node('text_to_speech', anonymous=True)
        self.text_to_speech = rospy.Service('/rip/turtlebot/voice/text_to_speech', TextToSpeech, self.callback)
        self.language = rospy.get_param('~language', 'en')
        self.slow = rospy.get_param('~slow', False)
        rospy.loginfo('TextToSpeech service is ready')

    def callback(self, req):
        text = req.text
        rospy.loginfo("Received text: %s", text)

        with tempfile.NamedTemporaryFile(delete=True) as audio_file:
            tts = gTTS(text, lang=self.language, slow=self.slow)
            tts.save(audio_file.name)
            os.system(f"mpg321 {audio_file.name}")

        res = TextToSpeechResponse()
        res.success = True
        return res

def main():
    try:
        TextToSpeechNode()
        rospy.spin()
    except rospy.ROSInitException:
        pass

if __name__ == '__main__':
    main()