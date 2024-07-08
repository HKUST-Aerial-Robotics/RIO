# This part will be released soon ...

## Prerequisites

Please refer to the `docker/Dockerfile` for the detailed dependencies.

## Dcoker (recommended)

### Start the docker container

```bash
./docker/docker.sh -b # build the docker image
./docker/docker.sh -r # run the docker container
```

### Outside the docker container

```bash
roscore &
rviz # config file: rio/config/RIO.rviz
```

### Inside the docker container

```bash
cd /ws
catkin_make

# Run the RIO with the sample dataset
python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/ars548.yaml -d /ws/src/dataset/exp/Sequence_1.bag -r 1 -p 1
```

Then you can see the odometry and the point cloud in rviz.

## Self collected dataset

- **Sequence 1** : involved relatively low-speed movements.

- **Sequence 2** : involved relatively high-speed movements.

- **Sequence 3** : involved relatively high-speed movements with high-speed rotations.

### Data Format

Please refer to the function `RIO::decodeRadarMsg_ARS548(const sensor_msgs::PointCloud2 &msg)` in `rio/node/rosWarper.cpp` for the detailed data format.