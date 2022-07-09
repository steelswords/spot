FROM ubuntu:18.04
RUN apt-get update && apt-get -y install g++
COPY . /app
RUN cd /app && /app/build.sh
ENTRYPOINT /app/gpio_test
