#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/proteenteen/Documents/DO-AN/DO-AN/ros/src/geographic_info/geodesy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/proteenteen/Documents/DO-AN/DO-AN/ros/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/proteenteen/Documents/DO-AN/DO-AN/ros/install/lib/python3/dist-packages:/home/proteenteen/Documents/DO-AN/DO-AN/ros/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/proteenteen/Documents/DO-AN/DO-AN/ros/build" \
    "/usr/bin/python3" \
    "/home/proteenteen/Documents/DO-AN/DO-AN/ros/src/geographic_info/geodesy/setup.py" \
    egg_info --egg-base /home/proteenteen/Documents/DO-AN/DO-AN/ros/build/geographic_info/geodesy \
    build --build-base "/home/proteenteen/Documents/DO-AN/DO-AN/ros/build/geographic_info/geodesy" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/proteenteen/Documents/DO-AN/DO-AN/ros/install" --install-scripts="/home/proteenteen/Documents/DO-AN/DO-AN/ros/install/bin"
