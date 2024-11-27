import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
import time
from queue import Queue
from threading import Thread
from audio_common_msgs.msg import AudioDataStamped, AudioInfo


class AudioRecorder(Node):
    def __init__(self):
        super().__init__("audio_driver")

        # Declare parameters
        self.declare_parameter("device", "")
        self.declare_parameter("bitrate", 128)
        self.declare_parameter("channels", 1)
        self.declare_parameter("sample_rate", 44100)
        self.declare_parameter("frame_id", "microphone")

        # Load parameters
        self.device = self.get_parameter("device").value
        self.channels = self.get_parameter("channels").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(AudioDataStamped, "audio_stamped", 10)
        self.info_publisher = self.create_publisher(AudioInfo, "info", 10)

        self.audio_queue = Queue()
        self.running = True

        self.timestamp_offset = None
        self.worker_thread = Thread(target=self.process_audio_queue)
        self.worker_thread.start()

        while True:
            sd._terminate()
            sd._initialize()
            device_names = [device['name'] for device in sd.query_devices()]
            if any(self.device in name for name in device_names):
                self.get_logger().info(f"Audio device '{self.device}' found.")
                break
            self.get_logger().warn(f"Audio device '{self.device}' not found in\n{str(device_names)}.")
            time.sleep(1.0)

    def record_audio(self):
        self.get_logger().info("Recording...")

        # Start recording in a separate thread
        def callback(indata, frames, time, status):
            if status:
                self.get_logger().warn(f"{time}: {status}")
            self.audio_queue.put((time, indata.copy()))

        try:
            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                device=self.device,
                callback=callback,
                dtype="int16",
                blocksize=self.sample_rate, # 1 second buffer
            ):
                while self.running:
                    rclpy.spin_once(self, timeout_sec=0.1)  # Keep the ROS 2 node alive
        except ValueError as e:
            self.get_logger().error(f"The ALSA device does not exist: {e}")
        finally:
            self.running = False
            self.worker_thread.join()

    def process_audio_queue(self):
        while self.running or not self.audio_queue.empty():
            if not self.audio_queue.empty():
                timestamp, data = self.audio_queue.get()
                if self.timestamp_offset is None:
                    self.timestamp_offset = self.get_clock().now().nanoseconds/1e9 - timestamp.inputBufferAdcTime

                self.publish_audio(timestamp.inputBufferAdcTime + self.timestamp_offset, data)

    def publish_audio(self, timestamp, data):
        msg = AudioDataStamped()
        msg.header.stamp.sec = int(timestamp)
        msg.header.stamp.nanosec = int((timestamp % 1) * 1e9)
        msg.header.frame_id = self.frame_id
        msg.audio.data = (data).tobytes()
        self.publisher.publish(msg)

        info_msg = AudioInfo()
        info_msg.channels = self.channels
        info_msg.sample_rate = self.sample_rate
        self.info_publisher.publish(info_msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    audio_recorder = AudioRecorder()

    try:
        audio_recorder.record_audio()
    except KeyboardInterrupt:
        pass

    audio_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
