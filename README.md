# srr308_radar_driver
This is the ROS2 based radar driver for Continental SRR308, which is based on Continental ARS408 driver from FH Aachen University of Applied Sciences[Lecture: Perception_Radar](https://gitlab.com/ApexAI/autowareclass2020/-/blob/bc1206171c96907977b0f5a8f4e3bc039ce61ae6/lectures/09_Perception_Radar/Radar-Hands-On.md) and Polymathrobotics.

## To run the Docker
```shell
docker run -it --rm \
  --network host \
  --ipc host \
  srr308-radar:latest
```