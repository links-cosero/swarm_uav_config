CONTAINER_NAME=humble_cc_container

# Import workspace
docker start $CONTAINER_NAME
docker exec $CONTAINER_NAME rm -rf /opt/rootfs/tmp_ws
docker exec $CONTAINER_NAME mkdir -p /opt/rootfs/tmp_ws
docker cp $PWD/src $CONTAINER_NAME:/opt/rootfs/tmp_ws
docker cp $PWD/workspace_cc $CONTAINER_NAME:/opt/rootfs/tmp_ws

# Cross compile
docker exec $CONTAINER_NAME bash -c "cd /opt/rootfs/tmp_ws && source /opt/rootfs/opt/ros2_humble/install/local_setup.bash && . workspace_cc/cross_compile_ws.bash"

# Compress and copy to host
docker exec $CONTAINER_NAME bash -c "tar -cf - /opt/rootfs/tmp_ws/install/ | gzip > /opt/rootfs/tmp_ws/my_pkg_install.tar.gz"
mkdir install_armhf
docker cp $CONTAINER_NAME:/opt/rootfs/tmp_ws/my_pkg_install.tar.gz ./install_armhf/
docker stop $CONTAINER_NAME
