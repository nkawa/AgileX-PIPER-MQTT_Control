from paho.mqtt import client as mqtt
from piper_sdk import *


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

import threading

load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))

class PiPER_CON:
    def __init__(self):
        self.last = 0
        self.average = np.ndarray((10,),np.dtype("int16"))
        self.average.fill(0)

    def init_piper(self):
        print("Initializing Piper control")
        #本当はシェアすべき？
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

    def main_loop(self):
        count=0
        print("[CNT]Start Main Loop")
        while self.loop:

            # 現在情報を取得しているかを確認
            if self.pose[0:6].sum() == 0:
                time.sleep(0.3)
                print("[CNT]Wait for monitoring..")
                continue

            if self.pose[7:14].sum() == 0:
                time.sleep(0.8)
                print("[CNT]Wait for target..")
                continue 
            
            now = time.time()
            if self.last == 0:
                self.last = now
                print("[CNT]Starting to Control!",self.pose)
                continue
            
            self.last = now
            joint_q = self.pose[7:14].tolist()  #これだと生の値
            
            # 各ジョイントへの制御をここで渡す
            print("[CNT]",joint_q)



    def run_proc(self):

        self.sm = mp.shared_memory.SharedMemory("PiPER")
        self.pose = np.ndarray((16,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.loop = True
        self.init_realtime()
        print("[CNT]:start realtime")
        self.init_piper()
        print("[CNT]:init PiPER")

        try:
            self.main_loop()
        except KeyboardInterrupt:
            print("[CNT]PiPER StopServo/Script")

            