xhost +si:localuser:root
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

export LIBGL_ALWAYS_INDIRECT=1

docker run -it \
    --name="u16_container" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/home/tinpl/Documents/vins:/tmp/vins" \
    --volume="/media/tinpl/Windows3/Downloads/SLAM_dataset:/tmp/slam_dataset" \
    --runtime=nvidia \
    danangcity45/ub:vins-mono \
    bash
