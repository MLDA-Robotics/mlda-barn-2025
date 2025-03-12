# Docker

## Build from Dockerfile

```shell
docker build . -t barn2025:latest --no-cache
```

## Tag and push to DockerHub

```
docker tag barn2025:latest mldarobotics/barn2025:latest
docker push mldarobotics/barn2025:latest
```

## Pull from the DockerHub

```shell
docker run --rm -dt --name barn2025 \
	--gpus all \
	-e DISPLAY="$DISPLAY" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ~/mlda-barn-2025:/jackal_ws/src/mlda-barn-2025 \
	barn2025:latest


docker run --rm -dt --name barn \
	--gpus all \
	-e DISPLAY="$DISPLAY" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	mldarobotics/barn2025:latest

docker run --rm -dt --name barn \
	-e DISPLAY=":1" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	barn2025:latest

```

## ROS

- Run environment
```shell
python run_rviz_kul.py --world_idx 300
rostopic hz /cmd_vel
rostopic hz /front/scan
```

- Compile ROS Setup
```shell
cd /jackal_ws
catkin_make
source devel/setup.bash
```

- Clear map

```shell
rosservice call /move_base/clear_costmaps "{}"
```
