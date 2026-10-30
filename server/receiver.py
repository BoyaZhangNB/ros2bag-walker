#!/usr/bin/env python3
"""Receive the walker camera stream from the Jetson.

The Jetson connects to this server over TCP and pushes JPEG frames
(see protocol.py). Received frames can be:
  * viewed live in a browser at http://<server>:<http-port>/   (default on)
  * re-published as a ROS 2 sensor_msgs/CompressedImage         (--ros)
  * shown in an OpenCV window                                    (--show)
  * written to disk as individual JPEGs                          (--save-dir)

Only the Python standard library is needed for the default (browser) mode.
"""
import argparse
import logging
import os
import socket
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from protocol import read_frame

log = logging.getLogger('receiver')


class FrameStore:
    """Holds the most recent frame and wakes up anyone waiting for a new one."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = -1
        self.stamp = 0.0
        self.count = 0

    def put(self, seq, stamp, jpeg):
        with self.cond:
            self.jpeg, self.seq, self.stamp = jpeg, seq, stamp
            self.count += 1
            self.cond.notify_all()

    def wait_newer(self, count, timeout=5.0):
        """Block until a frame newer than `count` arrives. Returns (count, jpeg) or None."""
        with self.cond:
            if not self.cond.wait_for(lambda: self.count > count, timeout=timeout):
                return None
            return self.count, self.jpeg


def make_stream_handler(store, sinks):
    class StreamHandler(socketserver.BaseRequestHandler):
        def handle(self):
            peer = '%s:%d' % self.client_address
            log.info('Jetson connected from %s', peer)
            self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            frames, last_report, last_seq = 0, time.monotonic(), None
            try:
                while True:
                    frame = read_frame(self.request)
                    if frame is None:
                        break
                    seq, sec, nanosec, jpeg = frame
                    stamp = sec + nanosec * 1e-9
                    store.put(seq, stamp, jpeg)
                    for sink in sinks:
                        sink(seq, sec, nanosec, jpeg)

                    frames += 1
                    now = time.monotonic()
                    if now - last_report >= 10.0:
                        # Age is only meaningful if both clocks are synced (chrony/NTP).
                        age_ms = (time.time() - stamp) * 1000.0
                        skipped = seq - last_seq - frames if last_seq is not None else 0
                        log.info('%.1f fps, %d KB/frame, age %.0f ms, %d skipped by sender',
                                 frames / (now - last_report), len(jpeg) // 1024,
                                 age_ms, max(skipped, 0))
                        frames, last_report, last_seq = 0, now, seq
                    elif last_seq is None:
                        last_seq = seq
            except (OSError, ValueError) as e:
                log.warning('Stream from %s failed: %s', peer, e)
            log.info('Jetson %s disconnected', peer)

    return StreamHandler


INDEX_HTML = b"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Walker camera</title>
<style>body{margin:0;background:#111;display:flex;align-items:center;justify-content:center;height:100vh}
img{max-width:100%;max-height:100vh}</style></head>
<body><img src="/stream" alt="waiting for camera..."></body></html>
"""


def make_http_handler(store):
    class HttpHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            log.debug('http: ' + fmt, *args)

        def do_GET(self):
            if self.path == '/':
                self.send_body(INDEX_HTML, 'text/html')
            elif self.path == '/snapshot.jpg':
                if store.jpeg is None:
                    self.send_error(503, 'no frame yet')
                else:
                    self.send_body(store.jpeg, 'image/jpeg')
            elif self.path == '/stream':
                self.stream_mjpeg()
            else:
                self.send_error(404)

        def send_body(self, body, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def stream_mjpeg(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            count = 0
            try:
                while True:
                    got = store.wait_newer(count)
                    if got is None:
                        continue
                    count, jpeg = got
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: %d\r\n\r\n' % len(jpeg))
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

    return HttpHandler


def make_ros_sink(topic, frame_id):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    import array

    rclpy.init()
    node = Node('walker_stream_receiver')
    pub = node.create_publisher(CompressedImage, topic, 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    log.info('Re-publishing frames on ROS 2 topic %s', topic)

    def sink(seq, sec, nanosec, jpeg):
        msg = CompressedImage()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = frame_id
        msg.format = 'jpeg'
        msg.data = array.array('B', jpeg)
        pub.publish(msg)

    return sink


def make_save_sink(directory):
    os.makedirs(directory, exist_ok=True)
    log.info('Saving frames to %s', directory)

    def sink(seq, sec, nanosec, jpeg):
        path = os.path.join(directory, f'{sec}.{nanosec:09d}_{seq:08d}.jpg')
        with open(path, 'wb') as f:
            f.write(jpeg)

    return sink


def show_window(store):
    """Runs on the main thread, since most GUI backends require it."""
    import cv2
    import numpy as np

    count = 0
    while True:
        got = store.wait_newer(count, timeout=0.1)
        if got is not None:
            count, jpeg = got
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow('walker camera', img)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break
    cv2.destroyAllWindows()


class ThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bind', default='0.0.0.0', help='address to listen on')
    ap.add_argument('--port', type=int, default=5600, help='TCP port the Jetson streams to')
    ap.add_argument('--http-port', type=int, default=8080, help='browser viewer port, 0 to disable')
    ap.add_argument('--show', action='store_true', help='show frames in an OpenCV window')
    ap.add_argument('--save-dir', help='write every received frame as a JPEG here')
    ap.add_argument('--ros', action='store_true', help='re-publish as a ROS 2 CompressedImage')
    ap.add_argument('--ros-topic', default='/walker/camera/image_raw/compressed')
    ap.add_argument('--frame-id', default='camera')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')

    store = FrameStore()
    sinks = []
    if args.ros:
        sinks.append(make_ros_sink(args.ros_topic, args.frame_id))
    if args.save_dir:
        sinks.append(make_save_sink(args.save_dir))

    stream_server = ThreadingTCPServer((args.bind, args.port), make_stream_handler(store, sinks))
    threading.Thread(target=stream_server.serve_forever, daemon=True).start()
    log.info('Waiting for the Jetson on %s:%d', args.bind, args.port)

    if args.http_port:
        http_server = ThreadingHTTPServer((args.bind, args.http_port), make_http_handler(store))
        http_server.daemon_threads = True
        threading.Thread(target=http_server.serve_forever, daemon=True).start()
        log.info('Live view at http://<this-host>:%d/', args.http_port)

    try:
        if args.show:
            show_window(store)
        else:
            threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        stream_server.shutdown()


if __name__ == '__main__':
    main()
