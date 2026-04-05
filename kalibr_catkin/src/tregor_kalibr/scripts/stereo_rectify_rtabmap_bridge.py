#!/usr/bin/env python3
"""
Публикует ректифицированное стерео (как ORB live) + CameraInfo с нулевыми D для RTAB-Map (вариант B).

Читает тот же YAML, что live_stereo_rectified.cpp: K, D, T, Camera.model, разрешение.
Строит карты через cv2.fisheye / cv2.stereoRectify — логика совпадает с C++ runner.
"""
from __future__ import annotations

import sys

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


def _load_rectify(path: str):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open rectify yaml: {path}")

    w = int(fs.getNode("Camera.width").real())
    h = int(fs.getNode("Camera.height").real())
    model = fs.getNode("Camera.model").string()
    baseline_m = float(fs.getNode("Stereo.baseline_m").real())

    kl = fs.getNode("LEFT.K").mat()
    dl = fs.getNode("LEFT.D").mat()
    kr = fs.getNode("RIGHT.K").mat()
    dr = fs.getNode("RIGHT.D").mat()
    t_full = fs.getNode("Stereo.T_c1_c2_raw").mat()

    fs.release()

    if kl is None or dl is None or kr is None or dr is None or t_full is None:
        raise RuntimeError("Rectify yaml: missing LEFT/RIGHT K,D or Stereo.T_c1_c2_raw")

    r = t_full[:3, :3].astype(np.float64)
    t = t_full[:3, 3:4].astype(np.float64)
    size = (w, h)

    if model == "equidistant":
        # Сигнатура как в live_stereo_rectified.cpp: CALIB_ZERO_DISPARITY, newImageSize, balance, fov_scale
        r1, r2, p1, p2, q = cv2.fisheye.stereoRectify(
            kl,
            dl,
            kr,
            dr,
            size,
            r,
            t,
            cv2.fisheye.CALIB_ZERO_DISPARITY,
            size,
            0.0,
            1.0,
        )
        map_l1, map_l2 = cv2.fisheye.initUndistortRectifyMap(
            kl, dl, r1, p1, size, cv2.CV_16SC2
        )
        map_r1, map_r2 = cv2.fisheye.initUndistortRectifyMap(
            kr, dr, r2, p2, size, cv2.CV_16SC2
        )
    elif model == "radtan":
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            kl,
            dl,
            kr,
            dr,
            size,
            r,
            t,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=-1,
            newImageSize=size,
        )
        map_l1, map_l2 = cv2.initUndistortRectifyMap(
            kl, dl, r1, p1, size, cv2.CV_16SC2
        )
        map_r1, map_r2 = cv2.initUndistortRectifyMap(
            kr, dr, r2, p2, size, cv2.CV_16SC2
        )
    else:
        raise RuntimeError(f"Unsupported Camera.model: {model}")

    return {
        "width": w,
        "height": h,
        "model": model,
        "baseline_m": baseline_m,
        "p1": p1,
        "p2": p2,
        "map_l1": map_l1,
        "map_l2": map_l2,
        "map_r1": map_r1,
        "map_r2": map_r2,
    }


def _camera_info(p: np.ndarray, header: Header, width: int, height: int) -> CameraInfo:
    msg = CameraInfo()
    msg.header = header
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    k = p[:, :3]
    msg.K = k.flatten().tolist()
    msg.R = np.eye(3, dtype=np.float64).flatten().tolist()
    msg.P = p.flatten().tolist()
    return msg


class StereoRectifyBridge:
    def __init__(self):
        rospy.init_node("stereo_rectify_rtabmap_bridge")

        yaml_path = rospy.get_param("~rectify_yaml", "")
        if not yaml_path:
            rospy.logfatal("~rectify_yaml is required")
            sys.exit(1)

        self._data = _load_rectify(yaml_path)
        self._w = self._data["width"]
        self._h = self._data["height"]
        self._p1 = self._data["p1"]
        self._p2 = self._data["p2"]
        self._map_l1 = self._data["map_l1"]
        self._map_l2 = self._data["map_l2"]
        self._map_r1 = self._data["map_r1"]
        self._map_r2 = self._data["map_r2"]
        self._baseline = self._data["baseline_m"]

        self._left_frame = rospy.get_param("~left_frame_id", "tregor_stereo_left_optical")
        self._right_frame = rospy.get_param("~right_frame_id", "tregor_stereo_right_optical")
        self._publish_tf = rospy.get_param("~publish_tf", True)
        self._output_gray = rospy.get_param("~output_mono8", True)

        left_topic = rospy.get_param("~left_topic", "/cam0/image_raw")
        right_topic = rospy.get_param("~right_topic", "/cam1/image_raw")
        queue = int(rospy.get_param("~queue_size", 25))
        slop = float(rospy.get_param("~approx_sync_slop", 0.05))

        pub_l = rospy.get_param("~pub_left", "/stereo/left/image_rect")
        pub_r = rospy.get_param("~pub_right", "/stereo/right/image_rect")
        pub_cli = rospy.get_param("~pub_left_camera_info", "/stereo/left/camera_info")
        pub_cri = rospy.get_param("~pub_right_camera_info", "/stereo/right/camera_info")

        self._pub_l = rospy.Publisher(pub_l, Image, queue_size=queue)
        self._pub_r = rospy.Publisher(pub_r, Image, queue_size=queue)
        self._pub_cli = rospy.Publisher(pub_cli, CameraInfo, queue_size=queue)
        self._pub_cri = rospy.Publisher(pub_cri, CameraInfo, queue_size=queue)

        self._bridge = CvBridge()
        self._static_tf = None
        if self._publish_tf:
            self._static_tf = tf2_ros.StaticTransformBroadcaster()

        sub_l = message_filters.Subscriber(left_topic, Image, queue_size=queue)
        sub_r = message_filters.Subscriber(right_topic, Image, queue_size=queue)
        sync = message_filters.ApproximateTimeSynchronizer(
            [sub_l, sub_r], queue_size=queue, slop=slop
        )
        sync.registerCallback(self._on_pair)

        if self._publish_tf:
            self._send_static_tf()

        rospy.loginfo(
            "stereo_rectify_rtabmap_bridge: %s -> %s, %s (model=%s, %dx%d, baseline_m=%.4f)",
            yaml_path,
            pub_l,
            pub_r,
            self._data["model"],
            self._w,
            self._h,
            self._baseline,
        )

    def _cv_from_img(self, msg: Image) -> np.ndarray:
        enc = msg.encoding
        if enc in ("mono8", "8UC1"):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        if enc in ("bgr8", "rgb8", "8UC3"):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding=enc)

    def _send_static_tf(self):
        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = self._left_frame
        ts.child_frame_id = self._right_frame
        ts.transform.translation.x = float(self._baseline)
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = 0.0
        ts.transform.rotation.x = 0.0
        ts.transform.rotation.y = 0.0
        ts.transform.rotation.z = 0.0
        ts.transform.rotation.w = 1.0
        self._static_tf.sendTransform(ts)

    def _to_gray(self, img: np.ndarray) -> np.ndarray:
        if img.ndim == 2:
            return img
        if img.shape[2] == 3:
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img.shape[2] == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        return img[:, :, 0]

    def _on_pair(self, msg_l: Image, msg_r: Image):
        try:
            im_l = self._cv_from_img(msg_l)
            im_r = self._cv_from_img(msg_r)
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, "cv_bridge: %s", e)
            return

        if im_l.shape[1] != self._w or im_l.shape[0] != self._h:
            rospy.logwarn_throttle(
                5.0,
                "Image size %sx%s != rectify yaml %sx%s; remap may be wrong",
                im_l.shape[1],
                im_l.shape[0],
                self._w,
                self._h,
            )

        if self._output_gray:
            im_l = self._to_gray(im_l)
            im_r = self._to_gray(im_r)

        rect_l = cv2.remap(im_l, self._map_l1, self._map_l2, interpolation=cv2.INTER_LINEAR)
        rect_r = cv2.remap(im_r, self._map_r1, self._map_r2, interpolation=cv2.INTER_LINEAR)

        stamp = msg_l.header.stamp
        h_l = Header(stamp=stamp, frame_id=self._left_frame)
        h_r = Header(stamp=stamp, frame_id=self._right_frame)

        out_enc = "mono8" if self._output_gray else "bgr8"
        try:
            out_l = self._bridge.cv2_to_imgmsg(rect_l, encoding=out_enc)
            out_r = self._bridge.cv2_to_imgmsg(rect_r, encoding=out_enc)
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, "cv_bridge out: %s", e)
            return

        out_l.header = h_l
        out_r.header = h_r

        ci_l = _camera_info(self._p1, h_l, self._w, self._h)
        ci_r = _camera_info(self._p2, h_r, self._w, self._h)

        self._pub_l.publish(out_l)
        self._pub_r.publish(out_r)
        self._pub_cli.publish(ci_l)
        self._pub_cri.publish(ci_r)


def main():
    try:
        StereoRectifyBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
