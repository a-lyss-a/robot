

#--------------------------------------------------------------
# Docker image build

#export GID = $(id -g)

buildimg:
	cd docker && GID=$$(id -g) docker-compose build

clean:
	rm -rf build install log




#--------------------------------------------------------------
# ROS2 development targets


run:
	cd docker &&  GID=$$(id -g) docker-compose up --force-recreate --remove-orphans


# Get a browser window with no decorations
vscode:
	/Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome --app=http://localhost:8080/?workspace=/home/dots/dots_system/dots.code-workspace --window-size=1600x900
screen:
	/Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome --app=http://localhost:8081 --window-size=1600x900