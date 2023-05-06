touch ./src/ros-perception/COLCON_IGNORE \
    ./src/ros-visualization/COLCON_IGNORE \
    ./src/ros/ros_tutorials/turtlesim/COLCON_IGNORE \
    ./src/ros2/demos/image_tools/COLCON_IGNORE \
    ./src/ros2/demos/intra_process_demo/COLCON_IGNORE \
    ./src/ros2/rviz/COLCON_IGNORE \
    ./src/ros2/rosbag2/COLCON_IGNORE \
    ./src/eclipse-iceoryx/COLCON_IGNORE

export C_INCLUDE_PATH="/opt/rootfs/usr/include:/opt/rootfs/usr/include/arm-linux-gnueabihf:/opt/rootfs/usr/lib/arm-linux-gnueabihf"
export CPLUS_INCLUDE_PATH="/opt/rootfs/usr/include:/opt/rootfs/usr/include/arm-linux-gnueabihf:/opt/rootfs/usr/lib/arm-linux-gnueabihf"
export LD_LIBRARY_PATH=/opt/rootfs/lib/arm-linux-gnueabihf:/opt/rootfs/lib
export PYTHON_EXECUTABLE=/opt/rootfs/usr/bin/python3.8
if [ ! -e /lib/ld-linux-armhf.so.3 ]
then
    ln -s /opt/rootfs/lib/ld-linux-armhf.so.3 /lib/ld-linux-armhf.so.3
fi
if [ ! -e /usr/include/eigen3 ]
then ln -s /opt/rootfs/usr/include/eigen3 /usr/include/eigen3
fi

colcon build \
    $@ \
    --merge-install \
    --cmake-force-configure \
    --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain.cmake \
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
    -DPYTHON_EXECUTABLE=/opt/rootfs/usr/bin/python3.8 \
    -DPYTHON_SOABI=cpython-38-arm-linux-gnueabihf \
    -DBUILD_TESTING:BOOL=OFF