import pymavlink.mavutil as utility     #/home/dronedev/.local/lib/python3.8/site-packages/pymavlink --> mavutil.py 
import pymavlink.dialects.v20.all as dialect #/home/dronedev/.local/lib/python3.8/site-packages/pymavlink/dialects/v20  -->all.py
import speech_recognition as sr
from gtts import gTTS
import os
import time
import playsound

#import sys #-- 텍스트 저장시 사용

#기체에 접속
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560")
#vehicle = utility.mavlink_connection('/dev/ttyUSB0', 115200)	#Telemetry # cd /dev/  여기서 맞는 포트 확인가능하세요
#vehicle = utility.mavlink_connection('/dev/ttyACM0', 57600)	#usb # cd /dev/ 여기서 자기컴에 맞는 포트 확인가능하세요
# 접속 신호 시간
vehicle.wait_heartbeat()

# 접속 기체의 SYSID SYSID>0 면 성공.
print("Connected to the vehicle")
print("Target system:", vehicle.target_system, "Target component:", vehicle.target_component)

def listen(questionv,lng):
    #obtain audio from the microphone
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print(questionv)
        r.adjust_for_ambient_noise(source, duration=0.5)
        audio = r.listen(source)
    #recognize speech using Google Speech Recognition

    try:
        var=r.recognize_google(audio,language=lng)
    except sr.UnknownValueError:
        var="Groot could not understand audio"
    except sr.RequestError:
        var=" Looks like, there is some problem with Google Speech Recognition"

    return var

def speak(text,spk_lng):

     tts = gTTS(text=text, lang=spk_lng)
     filename='voice.mp3'
     tts.save(filename)
     playsound.playsound(filename)

#sys.stdout = open('audio_output.txt', 'w') #-- 텍스트 저장시 사용

# arming
def mavlinkStr(cmdV , p1 ,p2 , p3, p4, p5, p6, p7):

    vehicle_arm_message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=cmdV,
        confirmation=0,
        param1=p1,
        param2=p2,
        param3=p3,
        param4=p4,
        param5=p5,
        param6=p6,
        param7=p7
    )
    vehicle.mav.send(vehicle_arm_message)


# eng kor 
englishYN = 1
if englishYN == 0:
    arming = "engine start"
    takeooffV = "take off"
    landV = "landing"
    audiosund = "tell me command"
    YnN = ["yes","no"]
    answersund = ""
    rightsund = " right?"
    armingsund = "arming now!"
    cntarmsund = "can't arming now!"
    lng = "en-US"
    spk_lng = "en"
else:
    arming = "시동"
    takeooffV = "이륙"
    landV = "착륙"
    audiosund = "명령을 내려주세요"
    YnN = ["네","아네요"]
    answersund = "다음명령을 실행할까요? "
    rightsund = " "
    armingsund = "드론시동 걸겠습니다."
    cntarmsund = "시동이 불가합니다."
    lng = "ko-KR"
    spk_lng = "ko"

while 1:

    #try:
        #audio = r.recognize_google(speech, language="ko-KR")
        audio =""
        speak(audiosund,spk_lng)
        audio = listen(audiosund,lng)
        if len(audio.strip()) != 32:
            
            #speak(answersund + audio + rightsund,spk_lng)
            #answer = listen(answersund + audio + rightsund,lng)
            
            #if len(answer.strip()) != 32:
            while 1: 
                    if audio==arming :
                        mavlinkStr(176,1,4,0,0,0,0,0)
                        print(armingsund)
                        speak(armingsund,spk_lng)
                        mavlinkStr(400,1,4,0,0,0,0,0)
                        
                        break
                    elif audio == takeooffV :
                        print(takeooffV)
                        speak(takeooffV,spk_lng)
                        mavlinkStr(22,0,0,0,0,0,0,10)
                        time.sleep(20)
                        break
                    elif audio == landV :
                        print(landV)
                        speak(landV,spk_lng)
                        mavlinkStr(dialect.MAV_CMD_NAV_LAND,0,0,0,0,0,0,0)
                        time.sleep(20)
                        break
                    else:
                        print(cntarmsund)
                        speak(cntarmsund,spk_lng)
                        break
            
        #except sr.UnknownValueError:
        #    print("Your speech can not understand")
        #except sr.RequestError as e:
        #    print("Request Error!; {0}".format(e)) 

        #sys.stdout.close() #-- 텍스트 저장시 사용