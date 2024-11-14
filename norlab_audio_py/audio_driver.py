import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
import time
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

        while True:
            # Check if the specified device is available
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
            self.publish_audio(indata.copy())

        try:
            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                device=self.device,
                callback=callback,
                dtype="float32",
                blocksize=4096,
            ):
                while True:
                    rclpy.spin_once(self)  # Keep the ROS 2 node alive
        except ValueError as e:
            self.get_logger().error(f"The ALSA device does not exist: {e}")

    def publish_audio(self, data):
        # Concatenate audio frames and convert to uint8
        msg = AudioDataStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.audio.data = (
            (data * 32767).astype(np.int16).tobytes()
        )  # Convert to int16 and then to bytes
        self.publisher.publish(msg)

        # Create and publish AudioInfo
        info_msg = AudioInfo()
        info_msg.channels = self.channels
        info_msg.sample_rate = self.sample_rate

        self.info_publisher.publish(info_msg)
        self.get_logger().debug("Published audio data and info.")


def main(args=None):
    rclpy.init(args=args)
    audio_recorder = AudioRecorder()

    # Start recording
    audio_recorder.record_audio()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
