### Installation

```bash
pip install -r requirements.txt
sudo apt install libportaudio2
```

You will also need the `audio_common_msgs` package.

### Usage
If you are unsure what device to use, you can run the following command to list all available devices:
```bash
ros2 run norlab_audio_py audio_driver
```
It will fail with a similar message:
```text
[audio_driver-1] ValueError: Multiple input devices found for '':
[audio_driver-1] [0] ATR4650-USB: USB Audio (hw:0,0), ALSA
[audio_driver-1] [1] ATR4650-USB: USB Audio (hw:1,0), ALSA
[audio_driver-1] [2] HDA Intel PCH: ALC888-VD Analog (hw:2,0), ALSA
```
In this case, you can specify the device you want to use by filling the `device` parameter with one of the three connected device:
```yaml
/**:
  audio_capture:
    ros__parameters:
      device: "hw:0,0"
```
