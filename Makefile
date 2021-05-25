

#--------------------------------------------------------------
# Docker image build


buildimg:
	docker-compose -f docker/docker-compose.yaml build

buildctrl:
	docker-compose -f docker/docker-compose.controller.yaml build)




clean:
	rm -rf build install log




#--------------------------------------------------------------
# ROS2 development targets


run:
	docker-compose -f docker/docker-compose.yaml up --remove-orphans

stop:
	docker compose -f docker/docker-compose.yaml down
	docker compose -f docker/docker-compose.controller.yaml down


runagent:
	docker-compose -f docker/docker-compose.controller.yaml up  --remove-orphans

run5agent:
	docker-compose -f docker/docker-compose.controller5.yaml up  --remove-orphans



# Get a browser window with no decorations (OSX only)
vscode:
	/Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome --app=http://localhost:8080/?workspace=/home/dots/dots_system/dots.code-workspace --window-size=1600x900
vncscreen:
	/Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome --app=http://localhost:8081 --window-size=1600x900
gzweb:
	/Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome --app=http://localhost:8085 --window-size=1600x900