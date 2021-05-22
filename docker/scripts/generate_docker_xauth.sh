#! /bin/sh
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.$USER.xauth nmerge -
