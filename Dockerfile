FROM alpine:latest AS shared-dependencies

RUN apk update && \
  apk add libtbb \
  yaml-cpp \
  openblas && \
  apk add -X http://dl-cdn.alpinelinux.org/alpine/edge/testing dlib

FROM shared-dependencies AS build-stage 

RUN apk update && \
  apk add g++ \
  cmake \
  make \
  eigen-dev \
  libtbb-dev \
  openblas-dev \
  yaml-cpp-dev && \
  apk add -X http://dl-cdn.alpinelinux.org/alpine/edge/testing dlib-dev

WORKDIR /home/dependencies
RUN wget https://github.com/jlblancoc/nanoflann/archive/refs/heads/master.zip
RUN unzip master.zip
RUN rm master.zip
WORKDIR ./nanoflann-master/build
RUN cmake ..
RUN make install

WORKDIR /home
COPY ./src ./src
WORKDIR /home/build
RUN cmake ../src
RUN make

FROM shared-dependencies AS final-stage

COPY --from=build-stage /home/build/simple ./simple

ENTRYPOINT ["./simple"]
