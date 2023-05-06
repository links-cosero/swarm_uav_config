docker buildx build\
	--tag test_ros2:armhf \
	--platform linux/arm/v7 \
	.

docker create -it \
	--platform linux/arm/v7 \
	--name ros2_test_armhf \
	test_ros2:armhf
