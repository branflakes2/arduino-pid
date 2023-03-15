all: build upload
build:
	arduino-cli compile -b Seeeduino:samd:seeed_XIAO_m0 .
upload:
	arduino-cli upload -b Seeeduino:samd:seeed_XIAO_m0 -p /dev/ttyACM0
