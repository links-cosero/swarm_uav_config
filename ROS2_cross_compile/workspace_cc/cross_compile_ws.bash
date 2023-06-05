# rootfs and ROS2 install locations
export ROOTFS_PATH=/opt/rootfs
export ROS2_INSTALL_PATH=$ROOTFS_PATH/opt/ros2_humble/install

export C_INCLUDE_PATH="$ROOTFS_PATH/usr/include:$ROOTFS_PATH/usr/include/arm-linux-gnueabihf:$ROOTFS_PATH/usr/lib/arm-linux-gnueabihf"
export CPLUS_INCLUDE_PATH="$ROOTFS_PATH/usr/include:$ROOTFS_PATH/usr/include/arm-linux-gnueabihf:$ROOTFS_PATH/usr/lib/arm-linux-gnueabihf"
export LD_LIBRARY_PATH=$ROOTFS_PATH/lib/arm-linux-gnueabihf:$ROOTFS_PATH/lib
export PYTHON_EXECUTABLE=$ROOTFS_PATH/usr/bin/python3.8

# Fix broken links
if [ ! -e /lib/ld-linux-armhf.so.3 ]
then
    ln -s $ROOTFS_PATH/lib/ld-linux-armhf.so.3 /lib/ld-linux-armhf.so.3
fi
if [ ! -e /usr/include/eigen3 ]
then ln -s $ROOTFS_PATH/usr/include/eigen3 /usr/include/eigen3
fi

# Check ROS 2 underlay
if [ -z $ROS_DISTRO ]
then 
    echo "Underlay not found. Execute before: source $ROS2_INSTALL_PATH/local_setup.bash"
    return
fi

colcon build \
    $@ \
    --merge-install \
    --cmake-force-configure \
    --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/workspace_cc/toolchain.cmake \
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
    -DPYTHON_EXECUTABLE=$ROOTFS_PATH/usr/bin/python3.8 \
    -DPYTHON_SOABI=cpython-38-arm-linux-gnueabihf \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_VERBOSE_MAKEFILE=ON
