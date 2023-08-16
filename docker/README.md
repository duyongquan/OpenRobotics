## 1 install docker

```bash
cd docker
./scripts/install_docker.sh
```



## 2 install `ros2 galactic` docker images

```bash
./run_dev.sh standalone.x86_64.dockerfile
```



##  3 run

```bash
xhost+
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY osrf/ros:galactic-desktop
```



docker run --env="DISPLAY" --net=host  --env="QT_X11_NO_MITSHM=1"  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -it  osrf/ros:galactic-desktop


docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY  --env="QT_X11_NO_MITSHM=1" osrf/ros:galactic-desktop

docker run --env="DISPLAY" --net=host --volume="$HOME/.Xauthority:/root/.Xauthority:rw"  --env="QT_X11_NO_MITSHM=1"  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -it --name visual_use -v /home:/out_home  osrf/ros:galactic-desktop



```
docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --runtime=nvidia \
    osrf/ros:galactic-desktop \
    bash
```