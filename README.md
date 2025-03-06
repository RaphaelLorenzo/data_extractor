# data_extractor

Scripts and software to extract recorded data from their native format (rosbags, video, pcaps...) to the desired database format.

## Installation

Install the docker with 

```
sh build_docker.sh
```

## Running for rosbags
```
sh run_docker.sh
cd /ros2_ws/src/data_extractor/
python3 extract_from_rosbag.py
```

## After recording, you get back ownership of the files and directories from the user
```
sudo chmod a+rwx ./saved_images/
sudo chmod a+rwx ./saved_images/*
sudo chmod a+rwx ./saved_point_clouds/
sudo chmod a+rwx ./saved_point_clouds/*
```
