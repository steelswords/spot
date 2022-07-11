# TODO: Is this the best base image?
FROM arm64v8/ros:foxy
RUN apt-get update && apt-get -y install g++ git
COPY . /app
RUN cd /app/ros_ws && \
       . /opt/ros/foxy/setup.sh && \
       rosdep install -i --from-path src --rosdistro foxy -y && \
       colcon build

