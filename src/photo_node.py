#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math, piexif, cv2, glob
from rospy import Service, ServiceProxy
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
from gs_interfaces.msg import PointGPS
from gs_interfaces.srv import NavigationSystem
from sensor_msgs.msg import Image

rospy.init_node("photo_node")
IMAGE_TOPIC = rospy.get_param(rospy.search_param("image_topic"))
FOLDER_PATH = rospy.get_param(rospy.search_param("folder_path"))

bridge = CvBridge()

photo_number = len(glob.glob(f"{FOLDER_PATH}/*.jpg"))

def pioneer2exif(data):
    d = int(math.modf(data)[1])
    md = math.modf(data)[0] * 60
    m = math.modf(md)[1]
    sd = math.modf(md)[0] * 60
    s = math.modf(sd)[1]
    return int(d), int(m), int(s)

def get_photo():
    global bridge
    img = rospy.wait_for_message(IMAGE_TOPIC, Image)
    return bridge.imgmsg_to_cv2(img,"bgr8")

def handle_make_photo(req):
    global photo_number
    global navigation_system_proxy
    cv2.imwrite(f"{FOLDER_PATH}/PIONEER_MAX_{photo_number:04d}.jpg", get_photo())
    if navigation_system_proxy().navigation == "GPS":
        position = rospy.wait_for_message("geoscan/navigation/global/position", PointGPS)
        lat_1, lat_2, lat_3 = pioneer2exif(position.latitude)
        lon_1, lon_2, lon_3 = pioneer2exif(position.longitude)
        
        exif_dict = {
            "0th":
            {
                piexif.ImageIFD.Make: u'Raspberry Pi Foundation',
                piexif.ImageIFD.Model : u'Raspberry Pi Camera Module v2',
                piexif.ImageIFD.Software: u'piexif',
            },
            "Exif":
            {
                piexif.ExifIFD.FocalLength: (304, 100),
                piexif.ExifIFD.FNumber: (2, 1)
            },
            "GPS":
            {
                piexif.GPSIFD.GPSLatitudeRef:"N",
                piexif.GPSIFD.GPSLatitude: ((lat_1,1), (lat_2,1), (lat_3,1)),
            
                piexif.GPSIFD.GPSLongitudeRef: "E",
                piexif.GPSIFD.GPSLongitude:((lon_1,1),(lon_2,1),(lon_3,1)),
                piexif.GPSIFD.GPSAltitudeRef: (0),
                piexif.GPSIFD.GPSAltitude: (int(position.altitude),1)
            }
        }

        piexif.insert(piexif.dump(exif_dict), f"{FOLDER_PATH}/PIONEER_MAX_{photo_number:04d}.jpg")
    photo_number += 1
    return EmptyResponse()

photo_service = Service("geoscan/make_photo", Empty, handle_make_photo)
navigation_system_proxy = ServiceProxy("geoscan/navigation/get_system", NavigationSystem)

while not rospy.is_shutdown():
    pass