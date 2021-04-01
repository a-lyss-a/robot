

#--------------------------------------------------------------
# Docker image build

#export GID = $(id -g)

build: docker/docker-compose.yaml docker/Dockerfile.rosdevel
	cd docker && GID=$$(id -g) docker-compose build

clean:
	rm -rf build install log




#--------------------------------------------------------------
# ROS2 development targets


run: docker/docker-compose.yaml docker/Dockerfile.rosdevel
	cd docker &&  GID=$$(id -g) docker-compose up --remove-orphans


# This needs to run as root to create the X display
run_gui:
	docker run -it --rm \
	-p 8080:8080 \
	-v `pwd`:/roshome/dev_ws:consistent \
	--name dev \
	--user="root" \
	rosdevel /roshome/scripts/start_foxy

run_gzweb:
	docker run -it --rm \
	-p 8080:8080 \
	-p 8081:8081 \
	-v `pwd`:/roshome/dev_ws:consistent \
	--name dev \
	rosdevel /roshome/scripts/start_foxy_gzweb

run_gui_nethost:
	docker run -it --rm \
	-p 8080:8080 \
	-v `pwd`:/roshome/dev_ws:consistent \
	--name dev \
	--net=host \
	rosdevel /roshome/scripts/start_foxy

dev_shell:
	docker exec -it --user="$$(id -u):$$(id -g)" -e HOME=/roshome dev bash \

screen:
	/Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome --app=http://localhost:8080 --window-size=1600x900
screenl:
	/Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome --app=http://192.168.0.8:8080 --window-size=1600x900