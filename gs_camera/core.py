#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
from crccheck.crc import Crc8DvbS2
from datetime import date

class RunCam():
    def __init__(self):
        self._resolution = ["4K@30FPS", "2.7K@60FPS", "2.7K@50FPS", "1080@120FPS", "1080@60FPS", "1080@50FPS", "1080@30FPS"]
        self._tv_mode = ["NTSC", "PAL"]
        self._serial = serial.Serial("/dev/ttyAMA3", baudrate = 115200, timeout = 2)

    def change_mode(self):
        package = (0xCC, 0x01, 0x02)
        code = Crc8DvbS2.calc(package)
        package += (code,)
        self._serial.write(package)

    def power_button(self):
        package = (0xCC, 0x01, 0x01)
        code = Crc8DvbS2.calc(package)
        package += (code,)
        self._serial.write(package)

    def close(self):
        self._serial.close()

    def __read_data(self):
        settings = b''
        while True:
            s = self._serial.read()
            settings += s
            if s == b'':
                break
        return settings

    def get_tv_mode(self):
        return self._tv_mode[int(str(self.__get_settings(2, 0)).split("\\")[5][2])]

    def get_resolution(self):
        return self._resolution[int(self.__get_settings(5,0)[4])]

    def get_sd_capacity(self):
        settings = str(self.__get_settings(3,0)).split("\\")[4].replace("x0c","").split("/")
        return {"free": int(settings[0]), "max": int(settings[1])}

    def get_remain_recording_time(self):
        return int(str(self.__get_settings(4,0)).split("\\")[4].replace("x0c",""))

    def get_camera_time(self):
        settings = str(self.__get_settings(6,0)).split("\\")[4].replace("n","")
        return {"time": float(settings[9:len(settings)]),"day":int(settings[6:8]) ,"month": int(settings[4:6]),"year": int(settings[0:4])}

    def __get_settings(self, settings_id = 0, chunk = 0):
        package = (0xCC, 0x11, settings_id, chunk)
        code = Crc8DvbS2.calc(package)
        package += (code,)
        self._serial.write(package)
        return self.__read_data()

    def set_resolution(self, resolution="4K@30FPS"):
        self.__set_settings(5, self._resolution.index(resolution))
        self.__read_data()

    def set_camera_time(self, day=date.today().day, month=date.today().month, year=date.today().year):
        time = str(year)
        if month < 10:
            time += "0" + str(month)
        else:
            time += str(month)
        if day < 10:
            time += "0" + str(day)
        else:
            time += str(day)
        time += "T0.0"
        self.__set_settings(6, tuple([ord(x) for x in time]))
        self.__get_settings(6, 0)

    def set_tv_mode(self, mode="NTSC"):
        self.__set_settings(2, tuple([self._tv_mode.index(mode)]))
        self.__read_data()
    
    def __set_settings(self,settings_id, body):
        package = (0xCC, 0x13,settings_id)
        package += body
        code = Crc8DvbS2.calc(package)
        package += (code,)
        self._serial.write(package)