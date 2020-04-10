FROM ubuntu:bionic

RUN mkdir -p /app
WORKDIR /app

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends g++ git ca-certificates make
RUN mkdir -p lib
RUN git clone https://gitlab.com/libeigen/eigen.git lib/eigen

# Add source & build
ADD ./src /app/src
ADD ./inc /app/inc
ADD ./Makefile /app/Makefile
RUN make

# Run the program.
CMD ./bin/cross-track
