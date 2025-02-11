import json
from paho.mqtt import client as mqtt
import multiprocessing as mp
import multiprocessing.shared_memory
import socket

from multiprocessing import Process, Queue

import sys
import os
from datetime import datetime
import math
import numpy as np
import time

## ここでUUID を使いたい
import uuid

import piper_monitor
import piper_control

from dotenv import load_dotenv
import ipget


load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))

MQTT_SERVER = os.getenv("MQTT_SERVER", "192.168.207.22")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "piper/vr")
ROBOT_UUID = os.getenv("ROBOT_UUID","no-uuid")
ROBOT_MODEL = os.getenv("ROBOT_MODEL","piper")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "dev")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID

#
#　MQTT ： Joint、もしくは ツール位置を受信する
#

"""
   メンテナンスフリーにするためには、 PiPER 側の状況を見守る必要がある。

  安定した実行のため、マルチプロセスを使う
    1. VRゴーグルからMQTT を受信するプロセス（目標値） 2 との間でQueue を利用
    2. CAN経由でPiPERを制御するプロセス（制御値）2との間で Queue, 3 との間で SharedMemory
    3. PiPER をモニタリングするプロセス
    　　→常に最新情報を提示　（SharedMemoryを利用）


"""

def get_ip_list():
    ll = ipget.ipget()
    flag = False
    ips = []
    for p in ll.list:
        if flag:
            flag=False
            if p == "127.0.0.1/8":
                continue
            ips.append(p)
        if p == "inet":
            flag = True
    return ips

class PiPER_MQTT:
    def __init__(self):
        self.start = -1
 #       self.log = open(fname,"w")

    def on_connect(self,client, userdata, flag, rc):
        print("MQTT:Connected with result code " + str(rc), "subscribe ctrl", MQTT_CTRL_TOPIC)  # 接続できた旨表示
        self.client.subscribe(MQTT_CTRL_TOPIC) #　connected -> subscribe

        # ここで、MyID Register すべき
        my_info = {
            "date" :  str(datetime.today()),
            "robotModel": ROBOT_MODEL,
            "codeType": "python-robot",
            "IP": get_ip_list(),
            "devId": ROBOT_UUID 
        }
        self.client.publish(MQTT_MANAGE_TOPIC+"/register", json.dumps(my_info))
        print("Publish",json.dumps(my_info))
        self.client.publish(MQTT_MANAGE_TOPIC+"/"+ROBOT_UUID, json.dumps({"date": str(datetime.today())}))

        self.client.subscribe(MQTT_MANAGE_RCV_TOPIC) #　connected -> subscribe

# ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def on_message(self,client, userdata, msg):
#        print("Message",msg.payload)
        if msg.topic == MQTT_CTRL_TOPIC:
            js = json.loads(msg.payload)
            try:
                if js['trigger'][1]== True: # A button
                    self.start = 0
                    print("Start controlling!")
                elif self.start < 0:
                    print("Waiting...for A button",js)
                    return

                if self.start > 100 and js['trigger'][1]==True: # stop controll?
                    self.start = -1
                    return
            except KeyError:
                print("No a button")
                print(js)

            self.start +=1

            rot =js["joints"]
#            print(rot)
            joint_q = [int(x*1000) for x in rot]
        # このjoint 情報も Shared Memoryに保存すべし！
            self.pose[8:15] = joint_q 
        # Target 情報を保存するだけ
        else:
            print("not subscribe msg",msg.topic)


    def connect_mqtt(self):

        self.client = mqtt.Client()  
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect(MQTT_SERVER, 1883, 60)
#  client.loop_start()   # 通信処理開始
        self.client.loop_forever()   # 通信処理開始

    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("PiPER")
        self.pose = np.ndarray((16,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.connect_mqtt()

class ProcessManager:
    def __init__(self):
        mp.set_start_method('spawn')
        sz = 32* np.dtype('float').itemsize
        try:
            self.sm = mp.shared_memory.SharedMemory(create=True,size = sz, name='PiPER')
        except FileExistsError:
            self.sm = mp.shared_memory.SharedMemory(size = sz, name='PiPER')
        self.ar = np.ndarray((16,), dtype=np.dtype("float32"), buffer=self.sm.buf) # 共有メモリ上の Array

    def startRecvMQTT(self):
        self.recv = PiPER_MQTT()
        self.recvP = Process(target=self.recv.run_proc, args=(),name="MQTT-recv")
        self.recvP.start()

    def startMonitor(self):
        self.mon = piper_monitor.PiPER_MON()
        self.monP = Process(target=self.mon.run_proc, args=(),name="PiPER-monitor")
        self.monP.start()

    def startControl(self):
        self.ctrl = piper_control.PiPER_CON()
        self.ctrlP = Process(target=self.ctrl.run_proc, args=(),name="PiPER-control")
        self.ctrlP.start()

## 共有メモリ(ar)でプロセス間通信を実施
##  0～6 までが、実アームのjoint 角度（6はグリッパ）7 はトルク
##  8～14までが、VRからの制御指令

    def checkSM(self):
        while True: ## ここで全体の状況を確認
            diff = self.ar[8:15]-self.ar[0:7]
#            diff *=1000
            diff = diff.astype('int')
            print(self.ar[:8],self.ar[8:15])
            print(diff)
            time.sleep(2)
    
                                

if __name__ == '__main__':
#        
    pm = ProcessManager()
    try:
        print("Monitor!")
        pm.startMonitor()
        print("MQTT!")
        pm.startRecvMQTT()
        print("Control")
        pm.startControl()
        print("Check!")
        pm.checkSM()
    except KeyboardInterrupt:
        print("Stop!")
#        self.sm.close()
#        self.sm.unlink()