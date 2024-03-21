# Robbert

Robbert is a program which is designed to detect 5 cm round balls of either aluminium or paper trash with imaged recognition, pick it up and drop it at a specific place.

## Recuirements

The followingen hardware is advised to use with Robbert:
- Raspberry Pi 4b, with [Mirte Software](https://docs.mirte.org/doc/install_mirte_software.html) installed.
- USB-webcam, connected to the Raspberry Pi USB 3.0 port
- Arduino Nano, connected to the Raspberry Pi USB 2.0 port with connected:
  - Servo (2)
  - Sonar sensor
  - Color sensor (2)

## Installation

Step 1. Update the Raspberry Pi

```bash
sudo apt-get update
sudo apt-get dist-upgrade
```

Step 2. Download this repository
This repository comes with a Tensorflow Lite model which is trained to recognize 5 cm balls of paper and aluminium. [This](https://github.com/EdjeElectronics/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi) will be a useful guide if you'd like to create your own TensorFlow Lite model.

```bash
git clone https://github.com/Programmeer-Beer/robbert.git
```

Step 3. Install TensorFlow Lite dependencies and OpenCV

```bash
cd robbert
bash get_pi_requirements.sh
```

Step 4. Create a config.py file and make changes to your liking

```bash
cp config.example.py config.py
```

Step 5. Run Robbert

```bash
python3 main.py
```

## Usage

Robbert should work flowlessly, but if you'd like to tinker a bit. Here's a overview of all the functions and what they do.

### robbert

With robbert you will be able to communicate with the Arduino. You can read the sensors and let it move its servo's.

```python
import robbert

# returns 'words'
robbert.move()
```

.HOE ROBBERT GEBRUIKEN?
Vind het dichtstbij zijnde object
object_closest['xpercentage'], object_closest['ypercentage'], object_closest['material'] = object_detection.closest()
return[0] = object_closest['xpercentage']: 0 <= xpercentage <= 100 [% vanaf links]
return[1] = object_closest['ypercentage']: 0 <= ypercentage <= 100 [% vanaf onder]
return[2] = object_closest['material']: 'aluminium' || 'papier'

Robbert laten bewegen
robbert.move(direction, speed, duration)
directions: 'forward' || 'backward' || 'right' || 'left'
speeds: 0 <= speed <= 100 [%]
duration: 0 <= duration [s]

Robbert laten oppakken
robbert.grab()

Robbert laten checken op potentiële botsing
robbert.collsion_warning()
return = warnings: True || False

Ontwijken berekenen
calculate.search_movement(object_closest)
return[0] = directions: 'forward' || 'backward' || 'right' || 'left'
return[1] = speeds: 0 <= speed <= 100 [%]
return[2] = duration: 0 <= duration [s]
return[3] = grab: True || False
