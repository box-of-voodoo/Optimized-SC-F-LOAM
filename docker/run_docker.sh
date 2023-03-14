
# docker run -it \                                                              --interactive, --tty
#     --net=host \                                                              same network seting as host
#     -v /tmp/.X11-unix:/tmp/.X11-unix \                                        to use X server of host
#     --env="DISPLAY" \                                                         to use X server of host
#     -h $HOSTNAME \                                                            to use X server of host
#     -v $HOME/.Xauthority:/home/jakub/.Xauthorit \                             to use X server of host, for authentication
#     -v $PWD/Optimized-SC-F-LOAM:/root/catkin_ws/src/Optimized-SC-F-LOAM \     mount location of code
#     -v /run/media/jakub/Data/datasets/diplomka/:/data \                       mount location of datasets
#     -v $PWD/../.bashrc:/root/.bashrc \                                        mount bashrc
#     $1 bash                                                                   first argument name of image to run; run bash on start
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --device=/dev/dri:/dev/dri \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$HOME/.Xauthority:/root/.Xauthorit" \
    -v "$PWD/../Optimized-SC-F-LOAM:/root/catkin_ws/src/Optimized-SC-F-LOAM" \
    -v "/media/jakub/Data/datasets/diplomka/:/data" \
    -v "$PWD/.bashrc:/root/.bashrc" \
    $1 sh -c "cd ~/catkin_ws;catkin_make;bash"