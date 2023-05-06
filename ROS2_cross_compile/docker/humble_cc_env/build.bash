docker buildx build\
	--tag humble_cc \
	.

docker create -it \
	--name humble_cc_container \
	humble_cc:latest
