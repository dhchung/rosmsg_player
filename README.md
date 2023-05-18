## Ros Message Player for Pohang Dataset

[![Example video](doc/youtube_preview.jpg)](https://youtu.be/VTmhBDnO0-o)
Example video

## Installation
Tested in ubuntu 20.04 with ROS noetic.

```sh
cd {your_catkin_ws}/src
git clone git@github.com:dhchung/rosmsg_player.git
cd .. && catkin_make
rosrun rosmsg_player rosmsg_player
```

Visualization
```sh
roscd rosmsg_player
rviz -d rosmsg_player.rviz
```


## Dataset structure

![Example video](doc/DataTree.jpg)
The code will work only if the dataset structure is configured as above figure.
Select the root folder of the dataset to load the data.

## Pohang Canal Dataset
Preprint of the paper: [arxiv](https://arxiv.org/abs/2303.05555).

## Acknowledgements
Inspired by IRAP file player (https://github.com/irapkaist/file_player).