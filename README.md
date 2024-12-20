# RIO

Optimization Based and Point Uncertainty Aware Radar-inertial Odometry for 4D Radar System

## Prerequisites

Please refer to the `docker/Dockerfile` for the detailed dependencies.

## Dcoker

### Start the docker container

```bash
./docker/docker.sh -b # build the docker image
./docker/docker.sh -r # run the docker container
```

### Outside the docker container

```bash
# Allow the docker container to connect to the X server
xhost +
```

### Inside the docker container

```bash
# In the first terminal
roscore &
rviz # config file: rio/config/RIO.rviz

# In the second terminal
cd /ws
catkin_make

# Run the RIO with the sample dataset
python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/ars548.yaml -d /ws/src/dataset/exp/Sequence_1.bag -r 1 -p 1
```

Then you can see the odometry and the point cloud in rviz.

## System Overview

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Galleryrio-factor.jpg)

## Experiment Platform

Our platform consists of a 4D FMCW Radar ARS548RDI manufactured by Continental and an IMU BMI088 manufactured by Bosch. The radar sensor is mounted on the front of the platform, while the IMU is mounted on the bottom.

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Galleryplatform.png)

## Trajectories on self-collected and ColoRadar dataset

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Galleryablation_pic.png)

**Red** trajectory is the proposed full system. **Blue** one is the system without point uncertainty model, and **black** one is the ground truth trajectory. We present the results on four sequences in two different datasets.

## Self-collected dataset

- **Sequence 1** : involved relatively low-speed movements.

- **Sequence 2** : involved relatively high-speed movements.

- **Sequence 3** : involved relatively high-speed movements with high-speed rotations.

### Data Format


| Field Name   | Data Type                        | Count | Offset (Bytes) | Remarks                        |
|--------------|----------------------------------|-------|----------------|--------------------------------|
| azimuth      | `sensor_msgs::PointField::FLOAT32` | 1     | 0              | Angle in the horizontal plane  |
| azimuthSTD   | `sensor_msgs::PointField::FLOAT32` | 1     | 4              | Standard deviation of azimuth  |
| elevation    | `sensor_msgs::PointField::FLOAT32` | 1     | 8              | Angle in the vertical plane    |
| elevationSTD | `sensor_msgs::PointField::FLOAT32` | 1     | 12             | Standard deviation of elevation|
| range        | `sensor_msgs::PointField::FLOAT32` | 1     | 16             | Distance to the target         |
| rangeSTD     | `sensor_msgs::PointField::FLOAT32` | 1     | 20             | Standard deviation of range    |
| velocity     | `sensor_msgs::PointField::FLOAT32` | 1     | 24             | Speed of the target            |
| velocitySTD  | `sensor_msgs::PointField::FLOAT32` | 1     | 28             | Standard deviation of velocity |
| rcs          | `sensor_msgs::PointField::INT8`    | 1     | 32             | Radar cross-section            |


## ColoRadar dataset

It consists of 52 sequences, recorded in mines, built environments, and in an urban creek path, totaling more than 145 minutes of 3D FMCW radar, 3D lidar, and IMU data. The full dataset, including sensor data, calibration sequences, and evaluation scripts. It is available at [ColoRadar](https://arpg.github.io/coloradar/).

## Citation

If you find our work useful in your research, please consider citing:

```bibtex
@article{huang2024morephysicalenhancedradarinertialodometry,
      title={Less is More: Physical-enhanced Radar-Inertial Odometry},
      author={Qiucan Huang and Yuchen Liang and Zhijian Qiao and Shaojie Shen and Huan Yin},
      booktitle={ICRA},
      year={2024},
}
```

```bibtex
@misc{xu2024modelingpointuncertaintyradar,
      title={Modeling Point Uncertainty in Radar SLAM},
      author={Yang Xu and Qiucan Huang and Shaojie Shen and Huan Yin},
      year={2024},
      eprint={2402.16082},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2402.16082},
}
```

## License

MIT License (see [LICENSE](LICENSE)).