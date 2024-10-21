import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
from audio_common_msgs.msg import AudioDataStamped, AudioInfo


class AudioRecorder(Node):
    def __init__(self):
        super().__init__("audio_recorder")

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

    def record_audio(self):
        self.get_logger().info("Recording...")

        # Start recording in a separate thread
        def callback(indata, frames, time, status):
            if status:
                self.get_logger().warn(status)
            print("indata: ", len(indata))
            self.publish_audio(indata.copy())

        print("sample rate: ", self.sample_rate)
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

    def publish_audio(self, data):
        # Concatenate audio frames and convert to uint8
        print(len(data))
        msg = AudioDataStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.audio.data = (
            (data * 32767).astype(np.int16).tobytes()
        )  # Convert to int16 and then to bytes
        self.publisher.publish(msg)
        self.get_logger().info("Published audio data.")

        # Create and publish AudioInfo
        info_msg = AudioInfo()
        info_msg.channels = self.channels
        info_msg.sample_rate = self.sample_rate

        self.info_publisher.publish(info_msg)
        self.get_logger().info("Published audio info.")


def main(args=None):
    rclpy.init(args=args)
    audio_recorder = AudioRecorder()

    # Start recording
    audio_recorder.record_audio()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
