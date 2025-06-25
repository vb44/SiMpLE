FROM alpine:latest AS shared-dependencies

RUN apk update && \
  apk add --no-cache libtbb \
  yaml-cpp \
  openblas && \
  apk add --no-cache -X http://dl-cdn.alpinelinux.org/alpine/edge/testing dlib

FROM shared-dependencies AS build-stage 

RUN apk add --no-cache g++ \
  cmake \
  make \
  eigen-dev \
  libtbb-dev \
  openblas-dev \
  yaml-cpp-dev && \
  apk add --no-cache -X http://dl-cdn.alpinelinux.org/alpine/edge/testing dlib-dev

WORKDIR /home/dependencies
RUN wget https://github.com/jlblancoc/nanoflann/archive/refs/heads/master.zip && \
  unzip master.zip && \
  rm master.zip
WORKDIR ./nanoflann-master/build
RUN cmake .. && make install

WORKDIR /home
COPY ./simple ./simple
WORKDIR ./build
RUN cmake ../simple && make

FROM shared-dependencies AS final-stage

COPY --from=build-stage /home/build/simple ./simple

ENTRYPOINT ["./simple"]
