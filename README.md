# Bluetooth Player for ESP-WROOM-32
# =================================

Based on ESP-IDF A2DP-SINK example.

Implements Bluetooth legacy profile A2DP for audio stream reception, AVRCP for media information notifications, and I2S for audio stream output interface.

Applications such as bluetooth speakers can take advantage of this example as a reference of basic functionalities.

After pairing, the application saves the address of the A2DP Source and will attempt to connect to it automatically after any subsequent powerup or reboot.

## How to use

### Hardware Required

A2DP is supported by the ESP32-WROOM-32 modules which have ESP32 chips. Modules with ESP32S2 or ESP32S3 chips don't support legacy A2DP.

To play sound, you'll need a stereo DAC, a stereo audio amplifier, and two speakers. The application can be configured to use an internal 8-bit DAC for testing, but of course that will sound pretty terrible for regular use. 

Without audio hardware, the app will only show a count of audio data packets received silently.

For the I2S codec, pick whatever chip or board works for you; this code was written using a PCM5102 chip, but other I2S boards and chips will probably work as well. The default I2S connections are shown below, but these can be changed in menuconfig:

| ESP pin   | I2S signal   |
| :-------- | :----------- |
| GPIO22    | LRCK         |
| GPIO25    | DATA         |
| GPIO26    | BCK          |

If the internal DAC is selected, analog audio will be available on GPIO25 and GPIO26. The output resolution on these pins will always be limited to 8 bit because of the internal structure of the DACs.

Connect an LED anode to GPIO33 through a suitable resistor to ground to act as a bluetooth status indicator. It blinks in different patterns to indicate connected, unconnected, connecting, and paused status.

### Configure the project

```
idf.py menuconfig
```

* Choose external I2S codec or internal DAC for audio output, and configure the output PINs under A2DP Example Configuration

* Enable Classic Bluetooth and A2DP under **Component config --> Bluetooth --> Bluedroid Enable**

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output.

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

After the program is started, the example starts inquiry scan and page scan, awaiting being discovered and connected. Other bluetooth devices such as smart phones can discover a device named "ESP_SPEAKER". A smartphone or another ESP-IDF example of A2DP source can be used to connect to the local device.

Once A2DP connection is set up, there will be a notification message with the remote device's bluetooth MAC address like the following:

```
I (106427) BT_AV: A2DP connection state: Connected, [64:a2:f9:69:57:a4]
```

If a smartphone is used to connect to local device, starting to play music with an APP will result in the transmission of audio stream. The transmitting of audio stream will be visible in the application log including a count of audio data packets, like this:

```
I (120627) BT_AV: A2DP audio state: Started
I (122697) BT_AV: Audio packet count 100
I (124697) BT_AV: Audio packet count 200
I (126697) BT_AV: Audio packet count 300
I (128697) BT_AV: Audio packet count 400

```

Also, the sound will be heard if a loudspeaker is connected and possible external I2S codec is correctly configured. For ESP32 A2DP source example, the sound is noise as the audio source generates the samples with a random sequence.

## Troubleshooting
* For current stage, the supported audio codec in ESP32 A2DP is SBC. SBC data stream is transmitted to A2DP sink and then decoded into PCM samples as output. The PCM data format is normally of 44.1kHz sampling rate, two-channel 16-bit sample stream. Other SBC configurations in ESP32 A2DP sink is supported but need additional modifications of protocol stack settings.
* As a usage limitation, ESP32 A2DP sink can support at most one connection with remote A2DP source devices. Also, A2DP sink cannot be used together with A2DP source at the same time, but can be used with other profiles such as SPP and HFP.
