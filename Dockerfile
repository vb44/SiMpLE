FROM ubuntu:latest AS build-stage 

ARG dlib_ver=19.24

RUN apt update && apt upgrade -y && \
apt install --no-install-recommends -y \
wget \
bzip2 \
unzip \
cmake \
make \
g++ \
libeigen3-dev \
libnanoflann-dev \
libtbb-dev

WORKDIR /home/dependencies
RUN wget --no-check-certificate http://dlib.net/files/dlib-${dlib_ver}.tar.bz2
RUN tar -xf dlib-${dlib_ver}.tar.bz2
WORKDIR ./dlib-${dlib_ver}/build
RUN cmake ..
RUN make install

WORKDIR /home/dependencies
RUN wget --no-check-certificate https://github.com/jbeder/yaml-cpp/archive/refs/heads/master.zip
RUN apt install --no-install-recommends -y unzip
RUN unzip master.zip
WORKDIR ./yaml-cpp-master/build
RUN cmake ..
RUN make install

WORKDIR /home
COPY ./src ./src
WORKDIR /home/build
RUN cmake ../src
RUN make

FROM ubuntu:latest AS final-stage

RUN apt update && apt upgrade -y && \
apt install --no-install-recommends -y \
libtbb-dev
COPY --from=build-stage /home/build/simple ./simple

ENTRYPOINT ["./simple"]
