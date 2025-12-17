import rclpy
import struct
import time
import sys
import numpy as np
import asyncio
import threading
from get_dispose_serial.myserial import AsyncSerial_t


class dispose_serial_init:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, callback=None):
        
        self.user_callback = callback
        self.serial = AsyncSerial_t(port, baudrate)
        self.serial.startListening(self.data_callback)
        self.data_queue = asyncio.Queue()
        self.form_len_list = {
            b'\xAA': {"format": "<fff", "callback": self._callback_odom}
                    # 可以继续添加其他数据帧类型和对应的处理函数
            } 

         # 为每个类型注册处理协程
        self.handlers = {}
        for data_form in self.form_len_list:
            # 绑定参数
            self.handlers[data_form] = lambda frame, df=data_form: self.handle_frame(frame, df)
               # 创建事件循环和后台线程
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        # 启动后台协程处理队列
        asyncio.run_coroutine_threadsafe(self.process_queue(), self._loop)
    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()
    def write(self, data: bytes):
        self.serial.write(data)
    def data_callback(self, data: bytes):#解析数据帧
        """串口数据回调函数 — 当收到下位机数据时触发"""
        if not data :
            return
        first_type = data.find(b'\xFF')
        if first_type == -1 :
            return
        data_form = data[first_type+1:first_type+2]
        if not data_form in self.form_len_list:
            return
        form_len = struct.calcsize(self.form_len_list[data_form]["format"])
        if len(data)-first_type < form_len+2:
            return
        data_frame = data[first_type+2:first_type+form_len+2]
        self.data_queue.put_nowait((data_form, data_frame))
    async def process_queue(self):#回调分发
        while True:
            data_form, frame = await self.data_queue.get()
            callback = self.form_len_list[data_form]["callback"]
            if data_form in self.form_len_list:
                try:
                    if callback:
                        callback(struct.unpack(self.form_len_list[data_form]["format"], frame))
                except struct.error as e:
                    print(f"解包失败")
            else:
                print(f"Unknown data form: {data_form}")
    def _callback_odom(self, data):# 处理里程计数据帧
        x, y, yaw = data
        if self.user_callback:
            self.user_callback(x, y, yaw)
            
    '''有需要继续增加_callback'''
    