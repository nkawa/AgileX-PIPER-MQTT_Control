# monitoring PiPER
import time
from piper_sdk import *
import json

if __name__ == "__main__":
    piper = C_PiperInterface()
    piper.ConnectPort()
    
    print(piper.GetPiperFirmwareVersion())
#    quit()
    
    while True:
        # piper.SearchMotorMaxAngleSpdAccLimit(1,0x02)
        piper.SearchAllMotorMaxAngleSpd()
        # piper.SearchAllMotorMaxAccLimit()
        gaj = piper.GetArmJointMsgs() # 各ジョイントと取得時刻がわかる
        aj = gaj.joint_state
        
        gag = piper.GetArmGripperMsgs() #グリッパの情報
        gj = gag.gripper_state
        
        
#        print(armJoint) # これで角度が出せる
        myjt = {
            "ctime": gaj.time_stamp,           
            "joints": [aj.joint_1, aj.joint_2, aj.joint_3, aj.joint_4, aj.joint_5, aj.joint_6],
            "gripper": [gj.grippers_angle,gj.grippers_effort, gj.status_code]
        }
        print(json.dumps(myjt))
        # もにたした情報をファイルに書き出す
        
        
        # # print(piper.GetCurrentMotorMaxAccLimit())
        # print(piper.GetCurrentMotorAngleLimitMaxVel())
#        print(piper.GetAllMotorAngleLimitMaxSpd())
        # print(piper.GetAllMotorMaxAccLimit())
        time.sleep(0.1)