#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
from rospy import Subscriber
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Response, Flask, render_template
from threading import Thread
from time import sleep

bridge=CvBridge()
img=None
app = Flask(__name__)

def callback(data):
    global bridge
    global img
    try:
        frame=bridge.imgmsg_to_cv2(data,"bgr8")
        frame=cv2.resize(frame,(640,480))
        img=cv2.imencode('.jpg', frame)[1].tobytes()
    except Exception as e:
        rospy.loginfo(str(e))

def generate():
    global img
    while True:
        sleep(0.01)
        yield(b'--frame\r\n'+b'Content-Type: image/jpeg\r\n\r\n' + img + b'\r\n')

@app.route("/")
def video():
	return Response(generate(),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

def target():
    rospy.init_node("web_viewer_node", disable_signals=True)
    cam=rospy.get_param(rospy.search_param("camera_topic"))
    rospy.wait_for_message(cam,Image)
    Subscriber(cam,Image,callback)

Thread(target=target).start()
app.run(host=os.popen('ip addr show wlan0').read().split("inet ")[1].split("/")[0], port=8088, debug=True, threaded=True, use_reloader=False)