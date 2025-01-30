# monitoring PiPER
import time
from piper_sdk import *
import json

from paho.mqtt import client as mqtt

from datetime import datetime
import time

import os
import sys
import json
import psutil

import multiprocessing as mp
from multiprocessing import Process, Queue
import multiprocessing.shared_memory

import numpy as np

from dotenv import load_dotenv

load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))


MQTT_SERVER = os.getenv("MQTT_SERVER", "127.0.0.1")
MQTT_ROBOT_STATE_TOPIC= os.getenv("MQTT_ROBOT_STATE_TOPIC", "piper/real")


class PiPER_MON:
    def __init__(self,verbose=False):
        self.verbose= verbose
        self.joints = [0,0,0,0,0,0,0]

    def init_piper(self):
        self.piper = C_PiperInterface()
        self.piper.ConnectPort()
        # 接続チェックしてもいいけどね。
        

    def init_realtime(self):
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)


    def on_connect(self,client, userdata, flag, rc,proc):
        print("Connected MQTT MQTT with result code " + str(rc))  # 接続できた旨表示

# ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected MQTT MQTT disconnection.")

    def connect_mqtt(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()   # 通信処理開始
#        self.client.loop_forever()   # 通信処理開始

    def monitor_start(self):
        lastStatus = -1
        lastRuntimeState = -1
        last = 0
        while True:
            now = time.time()
            if last == 0:
                last = now

            if now-last > 0.3: #0.3秒に１回程度で送る

                gaj = self.piper.GetArmJointMsgs() # 各ジョイントと取得時刻がわかる
                aj = gaj.joint_state
        
                gag = self.piper.GetArmGripperMsgs() #グリッパの情報
                gj = gag.gripper_state
        #        print(armJoint) # これで角度が出せる
                self.joints = [aj.joint_1, aj.joint_2, aj.joint_3, aj.joint_4, aj.joint_5, aj.joint_6, gj.grippers_angle]
                self.pose[8] = gj.grippers_effort #トルクの共有
                myjt = {
                    "ctime": gaj.time_stamp,           
                    "joints": self.joints,
                    "gripper": [gj.grippers_effort, gj.status_code]
                }
                self.client.publish(MQTT_ROBOT_STATE_TOPIC, json.dumps(myjt))
                last = now
            else:
                print("Now-last:",now, last, now-last)
            # ここで SharedMemory を使う！

            if self.verbose:
                n = time.time()
                print(n,myjt)
            self.pose[:7] = self.joints
            time.sleep(0.300)


    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("PiPER")
        self.pose = np.ndarray((16,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.init_realtime()
        self.init_piper()
        self.connect_mqtt()
        try:
            self.monitor_start()
        except KeyboardInterrupt:
            print("Stop! PiPER monitor")

if __name__ == '__main__':
    verbose = False
    if len(sys.argv)>1:
        if sys.argv[1]=="-v":
            verbose = True
    pp = PiPER_MON(verbose)
    pp.init_realtime()
    pp.init_piper()
    pp.connect_mqtt()

    try:
        pp.monitor_start()
    except KeyboardInterrupt:
        print("Monitor Main Stopped")
