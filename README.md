# ros2bag-walker
Stream camera to the cyborg collective.

An ethernet IP camera is read on a Jetson Orin Nano, published to ROS 2,
recorded to a rosbag on the Jetson, and streamed live to a remote server on the same network.

```
 IP camera ──RTSP──▶ Jetson Orin Nano                         Remote server
                     ┌──────────────┐  /camera/image_raw/     ┌──────────────┐
                     │ camera_node  │──compressed──┬─────────▶│ receiver.py  │──▶ browser :8080
                     │ (HW decode)  │              │  TCP     │              │──▶ ROS 2 topic (--ros)
                     └──────────────┘              │  :5600   └──────────────┘──▶ JPEGs (--save-dir)
                                         ┌─────────┴──┐  ▲
                                         │ ros2 bag   │  │
                                         │ record     │  stream_sender
                                         └────────────┘
```

* `jetson/ros2_ws/src/walker_camera`: ROS 2 (Humble) package that runs on the Jetson
  * `camera_node` opens the camera's RTSP stream through GStreamer, decoding with
    `nvv4l2decoder` in hardware, and publishes `sensor_msgs/CompressedImage` (JPEG) on
    `/camera/image_raw/compressed`. Raw `bgr8` on `/camera/image_raw` is optional.
  * `stream_sender` pushes those JPEGs to the server over TCP. It always sends the newest
    frame, so a slow network drops frames instead of adding delay. It reconnects on its own.
  * `walker.launch.py` starts both nodes plus `ros2 bag record`.
* `server/`: plain Python receiver that needs no ROS, apart from the optional `--ros` flag.

The stream uses a small TCP protocol (see `protocol.py`) instead of DDS. DDS is often
unreliable for large images on shared or Wi-Fi networks, and this way the server doesn't
need ROS installed. If the server runs ROS 2, `--ros` re-publishes the frames as a topic there.

## Camera

The code assumes a standard RTSP IP camera sending H.264 or H.265. To find its stream URL,
see [Finding the camera URL](#finding-the-camera-url) below.

Test the camera on the Jetson with GStreamer directly before involving ROS:

```bash
gst-launch-1.0 rtspsrc location="rtsp://..." latency=0 protocols=tcp ! \
  rtph264depay ! h264parse ! nvv4l2decoder ! fakesink -v
```

If the camera is GigE Vision (an industrial camera without RTSP), this RTSP path won't work.
In that case set `gst_pipeline` to an Aravis pipeline (`aravissrc ! ... ! appsink`).

### Network

If the camera is plugged straight into the Jetson's ethernet port, give that port a
static address on the camera's subnet, for example:

```bash
sudo nmcli con add type ethernet ifname eth0 con-name camera ipv4.method manual ipv4.addresses 192.168.1.10/24
```

### Finding the camera URL

Run these on the Jetson once the camera is plugged in and the port has an address.

**1. Find the camera's IP.** Check the label on the camera or its manual for the factory
default first (Hikvision uses `192.168.1.64`, Dahua `192.168.1.108`, many others
`192.168.1.10` or `.100`). Otherwise, scan the subnet:

```bash
sudo apt install nmap arp-scan
sudo arp-scan --interface=eth0 --localnet     # lists every device on eth0 with its MAC vendor
nmap -sn 192.168.1.0/24                       # ping sweep, if arp-scan finds nothing
```

If nothing shows up, the camera may be on a different subnet than the Jetson's port. Watch
for its traffic while you power-cycle it, then give `eth0` an address on that subnet:

```bash
sudo tcpdump -i eth0 -n arp or udp     # the camera's own IP appears as the sender
```

**2. Check that it serves RTSP.**

```bash
nmap -p 80,443,554,8554 <camera-ip>    # 554 or 8554 open = RTSP
```

**3. Find the stream path.** The easiest way is the camera's web UI, which usually shows the
RTSP URL under network or stream settings. If the Jetson is headless, tunnel the UI to your
laptop and open `http://localhost:8081`:

```bash
ssh -L 8081:<camera-ip>:80 <user>@<jetson-ip>     # run this on your laptop
```

Most cameras also support ONVIF, which reports the URL directly:

```bash
pip3 install --user onvif-zeep
python3 - <<'EOF'
from onvif import ONVIFCamera
cam = ONVIFCamera('<camera-ip>', 80, '<user>', '<pass>')
media = cam.create_media_service()
for p in media.GetProfiles():
    req = {'StreamSetup': {'Stream': 'RTP-Unicast', 'Transport': {'Protocol': 'RTSP'}},
           'ProfileToken': p.token}
    print(p.Name, media.GetStreamUri(req).Uri)
EOF
```

Failing both, try the common paths until one answers:

```bash
sudo apt install ffmpeg
for path in stream1 h264/ch1/main/av_stream cam/realmonitor?channel=1\&subtype=0 \
            live/ch00_0 live h264 11 Streaming/Channels/101; do
  echo "== $path"
  timeout 5 ffprobe -v error -rtsp_transport tcp \
    "rtsp://<user>:<pass>@<camera-ip>:554/$path" -show_entries stream=codec_name,width,height
done
```

**4. Note the codec.** `ffprobe` prints `codec_name=h264` or `hevc`. Pass `codec:=h265` to
the launch file for `hevc`. A profile with a lower resolution (often `stream2`, `subtype=1`
or channel `102`) is useful if the network is slow.

The server must be reachable from the Jetson on TCP port 5600, over Wi-Fi or a second
interface. Sync both clocks (`chrony`) if you want the latency figure in the server log
to be meaningful.

## Jetson setup (JetPack 6 / Ubuntu 22.04 / ROS 2 Humble)

```bash
sudo apt install ros-humble-ros-base ros-humble-rosbag2 python3-colcon-common-extensions
# Optional, for --storage mcap:
sudo apt install ros-humble-rosbag2-storage-mcap
```

Use the OpenCV that ships with JetPack (`python3-opencv`), **not** `pip install opencv-python`.
The pip wheel is built without GStreamer and can't open the pipeline. To check:

```bash
python3 -c "import cv2; print([l for l in cv2.getBuildInformation().splitlines() if 'GStreamer' in l])"
# should print: GStreamer: YES
```

Build:

```bash
cd jetson/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

On the server, start this first. It's fine to start it later too, since the Jetson keeps retrying:

```bash
cd server
python3 receiver.py                 # live view at http://<server-ip>:8080/
python3 receiver.py --ros           # also publish /walker/camera/image_raw/compressed
python3 receiver.py --show          # OpenCV window (pip install -r requirements.txt)
python3 receiver.py --save-dir out  # write every frame as a JPEG
```

On the Jetson:

```bash
ros2 launch walker_camera walker.launch.py \
    camera_url:=rtsp://admin:password@192.168.1.64:554/stream1 \
    server_host:=192.168.1.100
```

Launch arguments:

| arg | default | |
|---|---|---|
| `camera_url` | `rtsp://192.168.1.64:554/stream1` | camera RTSP URL |
| `codec` | `h264` | `h264` or `h265` |
| `server_host` / `server_port` | `192.168.1.100` / `5600` | where `receiver.py` listens |
| `stream` | `true` | send frames to the server |
| `record` | `true` | record a rosbag |
| `bag_dir` | `~/bags` | each run writes `walker_YYYYmmdd_HHMMSS/` |
| `storage` | `sqlite3` | or `mcap` |
| `max_bag_duration` | `300` | split the bag every N seconds |
| `topics` | `/camera/image_raw/compressed` | space-separated list to record |

Other settings (resolution, JPEG quality, raw publishing, stream fps cap, and so on) are in
`config/camera.yaml`.

The bag records JPEG frames, not raw images. Raw 1080p30 is about 190 MB/s, more than the
Orin Nano's storage can sustain. To play a bag back:

```bash
ros2 bag play ~/bags/walker_20261029_120000
```

## Troubleshooting

* **`Could not open camera`**: run the `gst-launch-1.0` command above. Then check the URL,
  the credentials, `codec`, and that OpenCV reports `GStreamer: YES`.
* **Choppy stream but a smooth bag**: the network is the bottleneck. Lower `max_fps` or
  `jpeg_quality` for the stream, or set the camera to a sub-stream with a lower resolution.
* **`nvv4l2decoder` missing**: set `hw_decode: false` to decode on the CPU, which is slower.
