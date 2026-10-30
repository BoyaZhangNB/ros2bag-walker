"""Read an ethernet IP camera over RTSP and publish it as ROS 2 images.

Decoding goes through GStreamer so the Jetson's hardware decoder (nvv4l2decoder)
does the heavy lifting. Frames are grabbed on a background thread and published
as sensor_msgs/CompressedImage (JPEG) and, optionally, raw sensor_msgs/Image.
"""
import array
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header


def build_gst_pipeline(url, codec, hw_decode, latency_ms, width, height):
    """Build a GStreamer pipeline string ending in a BGR appsink for OpenCV."""
    if codec not in ('h264', 'h265'):
        raise ValueError(f"codec must be 'h264' or 'h265', got {codec!r}")

    src = f'rtspsrc location="{url}" latency={latency_ms} protocols=tcp drop-on-latency=true'
    depay = f'rtp{codec}depay ! {codec}parse'
    size = f',width={width},height={height}' if width > 0 and height > 0 else ''

    if hw_decode:
        # nvvidconv does colour conversion and scaling on the VIC, not the CPU.
        decode = f'nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx{size} ! videoconvert'
    else:
        decode = f'avdec_{codec} ! videoscale ! videoconvert ! video/x-raw{size}'

    sink = 'video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false'
    return f'{src} ! {depay} ! {decode} ! {sink}'


def to_uint8_array(buf):
    # Assigning an array.array skips rosidl's per-element validation, which is
    # very slow for multi-megabyte images.
    return array.array('B', buf)


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_url', 'rtsp://192.168.1.64:554/stream1')
        self.declare_parameter('codec', 'h264')
        self.declare_parameter('hw_decode', True)
        self.declare_parameter('latency_ms', 0)
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        # If set, used verbatim instead of the generated pipeline. Must end in an appsink.
        self.declare_parameter('gst_pipeline', '')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('reconnect_delay_s', 2.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.frame_id = p('frame_id')
        self.jpeg_quality = int(p('jpeg_quality'))
        self.publish_raw = bool(p('publish_raw'))
        self.publish_compressed = bool(p('publish_compressed'))
        self.reconnect_delay = float(p('reconnect_delay_s'))

        self.pipeline = p('gst_pipeline') or build_gst_pipeline(
            p('camera_url'), p('codec'), bool(p('hw_decode')),
            int(p('latency_ms')), int(p('width')), int(p('height')))
        self.get_logger().info(f'GStreamer pipeline: {self.pipeline}')

        # Reliable so rosbag2 (which subscribes reliably by default) gets every frame.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.raw_pub = self.create_publisher(Image, 'camera/image_raw', qos)
        self.jpeg_pub = self.create_publisher(
            CompressedImage, 'camera/image_raw/compressed', qos)

        self.frames = 0
        self.last_report = time.monotonic()
        self.running = True
        self.thread = threading.Thread(target=self.grab_loop, daemon=True)
        self.thread.start()

    def grab_loop(self):
        while self.running and rclpy.ok():
            cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                self.get_logger().warn(
                    f'Could not open camera, retrying in {self.reconnect_delay:.1f}s '
                    '(is OpenCV built with GStreamer? see README)')
                time.sleep(self.reconnect_delay)
                continue

            self.get_logger().info('Camera stream opened')
            while self.running and rclpy.ok():
                ok, frame = cap.read()
                if not ok:
                    self.get_logger().warn('Lost camera stream, reconnecting')
                    break
                self.publish(frame)
            cap.release()
            time.sleep(self.reconnect_delay)

    def publish(self, frame):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        if self.publish_raw:
            msg = Image()
            msg.header = header
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = to_uint8_array(frame.tobytes())
            self.raw_pub.publish(msg)

        if self.publish_compressed:
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                msg = CompressedImage()
                msg.header = header
                msg.format = 'jpeg'
                msg.data = to_uint8_array(jpeg.tobytes())
                self.jpeg_pub.publish(msg)

        self.frames += 1
        now = time.monotonic()
        if now - self.last_report >= 10.0:
            fps = self.frames / (now - self.last_report)
            self.get_logger().info(
                f'{fps:.1f} fps, {frame.shape[1]}x{frame.shape[0]}')
            self.frames = 0
            self.last_report = now

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
