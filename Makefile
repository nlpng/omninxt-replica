kalibr-tools-melodic:
	@docker build -f ./kalibr-tools/Dockerfile.melodic -t quadcam-calib:melodic ./kalibr-tools

kalibr-tools-noetic:
	@docker build -f ./kalibr-tools/Dockerfile.noetic -t quadcam-calib:noetic ./kalibr-tools
