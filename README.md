## Ros Message Player for Pohang Dataset

Inspired by IRAP file player (https://github.com/irapkaist/file_player).
Special (and super) thanks for providing the codes.

[Example video](https://www.youtube.com/watch?v=Gn_xkNfslrc)

Plays ROS messages for the dataset folder composed as :

```
DataSetFolder
├── navigation
│   ├── gps.txt
│   └── ahrs.txt
├── stereo
│   ├── left_images
│   │   ├── 000000.png
│   │   └── ...
│   ├── right_images
│   │   ├── 000000.png
│   │   └── ...
│   └── timestamp.txt
├── infrared
│   ├── images
│   │   ├── 000000.png
│   │   └── ...
│   └── timestamp.txt
├── omni
│   ├── cam_0
│   │   ├── 000000.jpg
│   │   └── ...
│   ├── cam_1
│   │   ├── 000000.jpg
│   │   └── ...
│   ├── cam_2
│   │   ├── 000000.jpg
│   │   └── ...
│   ├── cam_3
│   │   ├── 000000.jpg
│   │   └── ...
│   ├── cam_4
│   │   ├── 000000.jpg
│   │   └── ...
│   ├── cam_5
│   │   ├── 000000.jpg
│   │   └── ...
│   └── timestamp.txt
├── lidar
│   ├── lidar_front
│   │   ├── points
│   │   │   ├── {rostime}.bin
│   │   │   └── ...
│   │   └── imu.txt
│   ├── lidar_port
│   │   ├── points
│   │   │   ├── {rostime}.bin
│   │   │   └── ...
│   │   └── imu.txt
│   └── lidar_starboard
│       ├── points
│       │   ├── {rostime}.bin
│       │   └── ...
│       └── imu.txt
├── radar
│   ├── images
│   │   ├── 000000.jpg
│   │   └── ...
│   └── timestamp.txt
└── calibration
    └── calibaration.json
```

Select the root folder of the dataset when loading (ex: {DataSetFolder} for the tree diagram above).

Tested in ubuntu 20.04 with ROS noetic.
