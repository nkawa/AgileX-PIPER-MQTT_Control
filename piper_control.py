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
        
        
    def enable_fun(self):
        enable_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while not(enable_flag):
            elapsed_time = time.time()-start_time
            print("----------")
            enable_flag = ( 
                self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
            )
            print("Status of PiPER:",enable_flag)
                
            self.piper.EnableArm(7)
            self.piper.GripperCtrl(0,1000,0x01, 0)
            print("----------")
            
            if elapsed_time > timeout:
                print("Timeout")
                elapsed_time_flag = True
                enable_flag - True
                break
            
            time.sleep(1)
        
        if elapsed_time_flag:
            print("Cannot enable the robot... for seconds ",timeout)
            exit(0)
                        

    def init_piper(self):
        print("Initializing Piper control")
        #本当はシェアすべき？
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        self.piper.EnableArm(7)
        self.gripper = 24000
        # 接続チェック
        time.sleep(0.5)
        self.enable_fun()

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
            if self.pose[0:7].sum() == 0:
                time.sleep(0.3)
                print("[CNT]Wait for monitoring..")
                continue

            if self.pose[8:15].sum() == 0:
                time.sleep(0.8)
                print("[CNT]Wait for joints..")
                continue 
            
            now = time.time()
            if self.last == 0:
                self.last = now
                print("[CNT]Starting to Control!",self.pose)
                continue
            
            self.last = now
            joint_q = self.pose[8:15].astype(int).tolist()  #これだと生の値
            current  = self.pose[0:7].astype(int)
            diff = self.pose[8:15]-self.pose[0:7]
            diff = diff.astype('int')

            # 各ジョイントへの制御をここで渡す
#            print("[CNT]",joint_q,diff)
            
            #アームの運動制御命令２
            # ctrl_mode: 0x00 待機、 0x01: CAN指令制御, 0x03:イーサネット, 0x04: WiFi, 0x07 オフライン
            # move_mode; 0x00 MoveP, 0x01: MoveJ, 0x02: MoveL, 0x03: MoveC, 0x04: Move M (Version 1.5-2 以降)
            # speed_rate:  0~100 の速度のパーセンテージ
            # residence_time: オフライン軌道点の滞留時間 (秒)
            # installation_pos: 設置位置 
            self.piper.MotionCtrl_2(0x01, 0x01, 40, 0x00)
            self.piper.JointCtrl(joint_q[0], joint_q[1], joint_q[2], joint_q[3], joint_q[4], joint_q[5])
            if self.gripper != joint_q[6]:
                self.piper.GripperCtrl(joint_q[6],200,0x01, 0)
                self.gripper = joint_q[6]
            
#            self.piper.JointCtrl(joint_q[0], joint_q[1], joint_q[2], current[3], current[4], current[5])
            #   print(self.piper.GetArmStatus())
            time.sleep(0.005)


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

            