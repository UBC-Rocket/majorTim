FROM debian:stable-slim

WORKDIR /home/rkt/build

RUN apt-get update && apt-get install -y \
	make \
	wget \
	bzip2 \
	figlet \
	&& rm -rf /var/lib/apt/lists/*

RUN wget -O gcc-arm.tar.bz2 https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2?revision=375265d4-e9b5-41c8-bf23-56cbe927e156?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,7-2017-q4-major && \
tar xjf gcc-arm.tar.bz2 -C /opt; rm gcc-arm.tar.bz2

ENV PATH "$PATH:/opt/gcc-arm-none-eabi-7-2017-q4-major/bin"
