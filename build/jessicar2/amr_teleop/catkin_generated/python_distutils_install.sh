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

echo_and_run cd "/home/eric/catkin_ws/src/jessicar2/amr_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/eric/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/eric/catkin_ws/install/lib/python2.7/dist-packages:/home/eric/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/eric/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/eric/catkin_ws/src/jessicar2/amr_teleop/setup.py" \
     \
    build --build-base "/home/eric/catkin_ws/build/jessicar2/amr_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/eric/catkin_ws/install" --install-scripts="/home/eric/catkin_ws/install/bin"
