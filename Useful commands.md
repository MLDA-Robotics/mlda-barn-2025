# Docker

## Build from Dockerfile

```shell
docker build . -t barn:april1 --no-cache

sudo docker run --rm -dt --name barn \
	--gpus all \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	barn:april1

docker push mldarobotics/barn2024

```

## Tag and push to DockerHub

```
docker tag barn:april1 mldarobotics/barn2024:april1
docker push mldarobotics/barn2024:april1
```

## Pull from the DockerHub

```shell
docker run --rm -dt --name barn \
	--gpus all \
	-e DISPLAY="$DISPLAY" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ~/mlda-barn-2024:/jackal_ws/src/mlda-barn-2024 \
	mldarobotics/barn2024:april1


docker run --rm -dt --name barn \
	--gpus all \
	-e DISPLAY="$DISPLAY" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	mldarobotics/barn2024:april1

docker run --rm -dt --name barn \
	-e DISPLAY=":1" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	barn:april1

```

## ROS

- Run environment
```shell
python run_rviz_auto_start.py --world_idx 0
python run_rviz_kul.py --world_idx 0
python run_rviz_imit.py --world_idx 0
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
