#!/usr/bin/env python3
import os
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()
last_t = {}


def make_callback(name):
    def cb(msg):
        try:
            now = time.time()
            if now - last_t.get(name, 0) < 1.0:
                return
            last_t[name] = now
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            out_dir = rospy.get_param("~debug_dir", os.path.expanduser("~/kalibr_debug"))
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, "{}_latest.jpg".format(name))
            cv2.imwrite(path, img)
            rospy.loginfo_throttle(5.0, "preview {} -> {}".format(name, path))
        except CvBridgeError as e:
            rospy.logwarn(str(e))
    return cb


def main():
    rospy.init_node("kalibr_preview_saver", anonymous=True)
    debug_dir = rospy.get_param("~debug_dir", os.path.expanduser("~/kalibr_debug"))
    rospy.loginfo("kalibr_preview_saver debug_dir=%s", debug_dir)
    rospy.Subscriber("/cam0/image_raw", Image, make_callback("cam0"), queue_size=1)
    rospy.Subscriber("/cam1/image_raw", Image, make_callback("cam1"), queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
