"""Forward compressed camera frames to a remote server over TCP.

Only the newest frame is ever queued: if the network is slower than the camera,
frames are dropped rather than buffered, so latency stays bounded.
The connection is re-established automatically if the server goes away.
"""
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from walker_camera.protocol import pack_frame


class StreamSender(Node):

    def __init__(self):
        super().__init__('stream_sender')

        self.declare_parameter('server_host', '192.168.1.100')
        self.declare_parameter('server_port', 5600)
        self.declare_parameter('topic', 'camera/image_raw/compressed')
        self.declare_parameter('max_fps', 0.0)  # 0 = no limit
        self.declare_parameter('reconnect_delay_s', 2.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.server = (p('server_host'), int(p('server_port')))
        max_fps = float(p('max_fps'))
        self.min_period = 1.0 / max_fps if max_fps > 0 else 0.0
        self.reconnect_delay = float(p('reconnect_delay_s'))

        self.cond = threading.Condition()
        self.latest = None
        self.seq = 0
        self.sent = 0
        self.dropped = 0

        self.create_subscription(CompressedImage, p('topic'), self.on_image, qos_profile_sensor_data)

        self.running = True
        self.thread = threading.Thread(target=self.send_loop, daemon=True)
        self.thread.start()
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'Streaming {p("topic")} to {self.server[0]}:{self.server[1]}')

    def on_image(self, msg):
        with self.cond:
            if self.latest is not None:
                self.dropped += 1
            self.latest = (msg.header.stamp.sec, msg.header.stamp.nanosec, bytes(msg.data))
            self.cond.notify()

    def next_frame(self):
        with self.cond:
            while self.running and self.latest is None:
                self.cond.wait(timeout=0.5)
            frame, self.latest = self.latest, None
            return frame

    def send_loop(self):
        while self.running:
            try:
                sock = socket.create_connection(self.server, timeout=5.0)
            except OSError as e:
                self.get_logger().warn(
                    f'Cannot reach server {self.server[0]}:{self.server[1]} ({e}), '
                    f'retrying in {self.reconnect_delay:.1f}s',
                    throttle_duration_sec=30.0)
                time.sleep(self.reconnect_delay)
                continue

            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info('Connected to server')
            last_send = 0.0
            try:
                while self.running:
                    frame = self.next_frame()
                    if frame is None:
                        continue
                    now = time.monotonic()
                    if now - last_send < self.min_period:
                        continue
                    sec, nanosec, jpeg = frame
                    sock.sendall(pack_frame(self.seq, sec, nanosec, jpeg))
                    self.seq += 1
                    self.sent += 1
                    last_send = now
            except OSError as e:
                self.get_logger().warn(f'Connection to server lost ({e}), reconnecting')
            finally:
                sock.close()

    def report(self):
        self.get_logger().info(f'sent {self.sent / 10.0:.1f} fps, dropped {self.dropped} in last 10s')
        self.sent = 0
        self.dropped = 0

    def destroy_node(self):
        self.running = False
        with self.cond:
            self.cond.notify_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StreamSender()
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
