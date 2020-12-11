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

bridge = CvBridge()
img = None
app = Flask(__name__)
interface = "wlan0"
port = 8088

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
        try:
            yield(b'--frame\r\n'+b'Content-Type: image/jpeg\r\n\r\n' + img + b'\r\n')
        except TypeError:
            pass

@app.route("/")
def video():
	return Response(generate(),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

def init():
    global interface
    global port
    rospy.init_node("web_viewer_node", disable_signals=True)
    img_topic = rospy.get_param(rospy.search_param("image_topic"))
    try:
        interface = rospy.get_param(rospy.search_param("interface"))
    except:
        rospy.logwarn("Parameter \'interface'\ not specified. The interface is set to {}".format(interface))

    try:
        port = int(rospy.get_param(rospy.search_param("port")))
    except:
        rospy.logwarn("Parameter \'port\' not specified. The port is set to {}".format(port))
    Subscriber(img_topic,Image,callback)

Thread(target=init).run()
app.run(host = os.popen("ip addr show {}".format(interface)).read().split("inet ")[1].split("/")[0], port=port, debug=True, threaded=True, use_reloader=False)