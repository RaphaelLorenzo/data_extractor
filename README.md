# data_extractor

Scripts and software to extract recorded data from their native format (rosbags, video, pcaps...) to the desired database format.

## Installation

Install the docker with 

```
sh build_docker.sh
```

## Running for rosbags
On one side replay the rosbag :
```
sh run_docker.sh
cd /ros2_ws/src/data_extractor/
ros2 bag play ./data/rosbag2_pepr_dummy/rosbag2_2025_03_04-13_59_55_0.db3
```

On the other side, run the script to extract the data :
``` 
sh run_docker.sh
cd /ros2_ws/src/data_extractor/
python3 extract_from_rosbag.py
```

## After recording

Get back ownership of the files and directories from the user

```
sudo chmod a+rwx ./saved_images/
sudo chmod a+rwx ./saved_images/*
sudo chmod a+rwx ./saved_point_clouds/
sudo chmod a+rwx ./saved_point_clouds/*
sudo chmod a+rwx ./saved_scale_weights/
sudo chmod a+rwx ./saved_scale_weights/*
```

## Visualization example

![gifexample](./images/visualization_animation.gif)