#!/usr/bin/env python3
from playsound import playsound
import rospy
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from play_tunes.srv import playTune, playTuneResponse, playTuneRequest

class AudioService:
    def __init__(self) -> None:
        self.play_tune = rospy.Service("playTune", playTune, self.play_tune_callback)

    
    def play_tune_callback(self,req:playTuneRequest) ->playTuneResponse:
        req = req.tuneToPlay.data
        req = req.lower()
        # detected objects
        if req == "lebronjames":
            self.play_lebron()
            return EmptyResponse()
        elif req == "binky":
            self.play_binky()
            return EmptyResponse()
        elif req == "kiki":
            self.play_kiki()
            return EmptyResponse()
        elif req == "hugo":
            self.play_hugo()
            return EmptyResponse()
        elif req == "slush":
            self.play_slush()
            return EmptyResponse()
        elif req == "oakie":
            self.play_oakie()
            return EmptyResponse()
        elif req == "muddles":
            self.play_muddles()
            return EmptyResponse()
        elif req == "greenball":
            self.play_greenball()
            return EmptyResponse()
        elif req == "blueball":
            self.play_blueball()
            return EmptyResponse()
        elif req == "redball":
            self.play_redball()
            return EmptyResponse()
        elif req == "bluecube":
            self.play_bluecube()
            return EmptyResponse()
        elif req == "redcube":
            self.play_redcube()
            return EmptyResponse()
        elif req == "greencube":
            self.play_greencube()
            return EmptyResponse()
        elif req == "woodencube":
            self.play_woodencube()
            return EmptyResponse()
        #memes
        elif req == "ugh":
            self.play_ugh()    
            return EmptyResponse()
        elif req == "agh":
            self.play_agh()
            return EmptyResponse()
        elif req == "gothim":
            self.play_gothim()
            return EmptyResponse()
        elif req == "!":
            self.playMetalGear()
            return EmptyResponse()
        elif req == "siu":
            self.play_siu()
            return EmptyResponse()

        else: 
            rospy.logerr("Invalid tune name")
            return EmptyResponse()
        
    #sound playing functions    
    #plushies
    def play_binky(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/binky.wav")
    def play_kiki(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/kiki.wav")
    def play_hugo(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/hugo.wav")
    def play_slush(self) ->None:  
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/slush.wav")
    def play_oakie(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/oakie.wav")
    def play_muddles(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/muddles.wav")
    #balls
    def play_greenball(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/greenball.wav")
    def play_blueball(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/blueball.wav")
    def play_redball(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/redball.wav")
    #cubes
    def play_bluecube(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/bluecube.wav")
    def play_redcube(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/redcube.wav")
    def play_greencube(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/greencube.wav")
    def play_woodencube(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/woodencube.wav")
        


    #memes
    def play_lebron(self)->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/LebronJames.mp3")

    def play_ugh(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/RobloxUgh.mp3")

    def play_agh(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/agh.mp3")

    def play_gothim(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/HaGotHim.mp3")

    def playMetalGear(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/metalgearsolid.swf.mp3")
    def play_siu(self) ->None:
        playsound("/home/robot/Dopey-DD2419/src/play_tunes/audio_files/justSiu.mp3")







if __name__ == "__main__":
    rospy.init_node("audio_service")
    AudioService()
    rospy.spin()