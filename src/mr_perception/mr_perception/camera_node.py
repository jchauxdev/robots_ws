"""Camara USB con OpenCV (V4L2) -> image_raw (+ image_raw/compressed JPEG) sin cv_bridge ni usb_cam.

Pensado para la Jetson Nano dentro del contenedor ROS 2 Humble (L4T r32.7.1), donde los paquetes apt
ros-humble-* no aplican. En el PC o en la AGX Orin tambien puede usarse usb_cam (apt).

  ros2 run mr_perception camera_node --ros-args -p device:=0 -p width:=640 -p height:=480 -p fps:=15
"""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('rgb_camera')
        p = self.declare_parameter
        dev = p('device', 0).value
        self.frame = p('frame_id', 'rgb_camera_optical_frame').value
        w, h, fps = int(p('width', 640).value), int(p('height', 480).value), float(p('fps', 15.0).value)
        self.quality = int(p('jpeg_quality', 80).value)
        self.publish_raw = bool(p('publish_raw', True).value)
        self.cap = cv2.VideoCapture(int(dev) if str(dev).isdigit() else dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # USB 2.0: MJPG permite 640x480@30
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError(f'No se pudo abrir la camara {dev} (revise: v4l2-ctl --list-devices)')
        self.raw_pub = self.create_publisher(Image, 'rgb_camera/image_raw', qos_profile_sensor_data)
        self.jpg_pub = self.create_publisher(CompressedImage, 'rgb_camera/image_raw/compressed',
                                             qos_profile_sensor_data)
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(f'Camara {dev}: {int(self.cap.get(3))}x{int(self.cap.get(4))} @ {fps} Hz')

    def tick(self):
        ok, img = self.cap.read()
        if not ok:
            self.get_logger().warn('Cuadro perdido', throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()
        if self.publish_raw and self.raw_pub.get_subscription_count() > 0:
            m = Image(height=img.shape[0], width=img.shape[1], encoding='bgr8', is_bigendian=0,
                      step=img.shape[1] * 3, data=img.tobytes())
            m.header.stamp, m.header.frame_id = stamp, self.frame
            self.raw_pub.publish(m)
        if self.jpg_pub.get_subscription_count() > 0:
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                c = CompressedImage(format='jpeg', data=buf.tobytes())
                c.header.stamp, c.header.frame_id = stamp, self.frame
                self.jpg_pub.publish(c)


def main():
    rclpy.init()
    n = CameraNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.cap.release()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
