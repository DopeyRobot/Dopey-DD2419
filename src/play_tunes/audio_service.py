#!/usr/bin/env python3
from playsound import playsound
import rospy
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from play_tunes.srv import playTune, playTuneResponse, playTuneRequest
AUDIO_PATH = "/home/robot/dd2419_ws/src/play_tunes/audio_files/"
class AudioService:
    def __init__(self) -> None:
        self.play_tune = rospy.Service("playTune", playTune, self.play_tune_callback)

    def cleanupStr(self, req:str):
        req = req.split("_")[0]
        req = req.lower()
        req = req.replace("_", "")
        req = req.replace("-", "")
        req = req.replace (" ", "")
        print(req)
        return req
    
    def play_tune_callback(self,req:playTuneRequest) ->playTuneResponse:
        req = req.tuneToPlay.data
        req = self.cleanupStr(req)
        # detected objects
        #plushies
        if req == "binky":
            playsound(AUDIO_PATH+"binky.wav")
            return EmptyResponse()
        elif req == "kiki":
            playsound(AUDIO_PATH+"kiki.wav")
            return EmptyResponse()
        elif req == "hugo":
            playsound(AUDIO_PATH+"hugo.wav")
            return EmptyResponse()
        elif req == "slush":
            playsound(AUDIO_PATH+"slush.wav")
            return EmptyResponse()
        elif req == "oakie":
            playsound(AUDIO_PATH+"oakie.wav")
            return EmptyResponse()
        elif req == "muddles":
            playsound(AUDIO_PATH+"muddles.wav")
            return EmptyResponse()
        #balls
        elif req == "greenball":
            playsound(AUDIO_PATH+"greenball.wav")
            return EmptyResponse()
        elif req == "blueball":
            playsound(AUDIO_PATH+"blueball.wav")
            return EmptyResponse()
        elif req == "redball":
            playsound(AUDIO_PATH+"redball.wav")
            return EmptyResponse()
        #cubes
        elif req == "bluecube":
            playsound(AUDIO_PATH+"bluecube.wav")
            return EmptyResponse()
        elif req == "redcube":
            playsound(AUDIO_PATH+"redcube.wav")
            return EmptyResponse()
        elif req == "greencube":
            playsound(AUDIO_PATH+"greencube.wav")
            return EmptyResponse()
        elif req == "woodencube":
            playsound(AUDIO_PATH+"woodencube.wav")
            return EmptyResponse()
        #box
        elif req == "box":
            playsound(AUDIO_PATH+"box.mp3")
            return EmptyResponse()
        #memes
        elif req == "lebronjames":
            playsound(AUDIO_PATH+"LebronJames.mp3")
            return EmptyResponse()
        elif req == "ugh":
            playsound(AUDIO_PATH+"RobloxUgh.mp3")
            return EmptyResponse()
        elif req == "agh":
            playsound(AUDIO_PATH+"agh.mp3")
            return EmptyResponse()
        elif req == "gothim":
            playsound(AUDIO_PATH+"HaGotHim.mp3")
            return EmptyResponse()
        elif req == "!":
            playsound(AUDIO_PATH+"metalgearsolid.swf.mp3")
            return EmptyResponse()
        elif req == "siu":
            playsound(AUDIO_PATH+"justSiu.mp3")
            return EmptyResponse()
        elif req == "bombastic":
            playsound(AUDIO_PATH+"bombastic.mp3")
            return EmptyResponse()
        elif req == "underwater":
            playsound(AUDIO_PATH+"underwater.mp3")
            return EmptyResponse()
        else: 
            rospy.logerr("Invalid tune name")
            playsound(AUDIO_PATH+"noMichael.mp3")
            return EmptyResponse()




if __name__ == "__main__":
    rospy.init_node("audio_service")
    AudioService()
    rospy.spin()