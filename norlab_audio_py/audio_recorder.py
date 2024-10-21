import rclpy
from rclpy.node import Node
import numpy as np
import wave
from audio_common_msgs.msg import AudioDataStamped


class AudioRecorder(Node):
    def __init__(self):
        super().__init__("audio_recorder")

        # Declare parameters
        self.declare_parameter("format", "wave")  # "wave" or "mp3"
        self.declare_parameter("filename", "output")
        self.declare_parameter("sample_rate", 44100)
        self.declare_parameter("channels", 1)

        # Load parameters
        self.format = self.get_parameter("format").value
        self.filename = self.get_parameter("filename").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.channels = self.get_parameter("channels").value

        self.timestamp = ""

        # Subscriber to audio_stamped
        self.subscription = self.create_subscription(
            AudioDataStamped, "audio_stamped", self.audio_callback, 10
        )

        self.frames = []  # Store audio frames
        self.timer = self.create_timer(
            10.0, self.dump_wav_file
        )  # Timer for dumping every 10 seconds

    def audio_callback(self, msg):
        # Convert the incoming audio data (uint8) back to int16 and store the frames
        if self.timestamp == "":
            self.timestamp = msg.header.stamp
        data = np.frombuffer(msg.audio.data, dtype=np.int16)
        self.frames.append(data)
        self.get_logger().info(f"Received {len(data)} samples of audio.")

    def dump_wav_file(self):
        if not self.frames:
            self.get_logger().warn("No audio data to save.")
            return
        timestamp_first_msg = self.timestamp
        self.timestamp = ""

        # Flatten all frames into one continuous array
        audio_data = np.concatenate(self.frames, axis=0)

        if self.format == "wave":
            # Write the audio data to a WAV file
            file_path = f"{self.filename}_{timestamp_first_msg}.wav"
            with wave.open(file_path, "wb") as wav_file:
                wav_file.setnchannels(self.channels)
                wav_file.setsampwidth(2)  # Assuming 16-bit audio (int16)
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(audio_data.tobytes())

        else:
            self.get_logger().error(f"Unsupported audio format: {self.format}.")
            exit(1)

        self.get_logger().info(f"WAV file saved as {file_path}")

        # Clear frames after saving
        self.frames = []


def main(args=None):
    rclpy.init(args=args)
    audio_recorder = AudioRecorder()

    rclpy.spin(audio_recorder)

    audio_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
