FROM althack/ros2:humble-full
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN sudo apt update


# ARG USER_UID=1000
# ARG USER_GID=$USER_UID
ARG USERNAME=ros
# Create a non-root user
RUN apt-get update \
  && apt-get install -y --no-install-recommends sudo git-core bash-completion  \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt install -y \
  python3-colcon-common-extensions \
  python3-ament-package \
  ros-humble-rosidl-generator-cpp \
  ros-humble-rosidl-generator-py



ARG WORKSPACE=/workspace
WORKDIR ${WORKSPACE}
RUN chown -R ros:ros ${WORKSPACE}

USER ros
# Make sure bash (for the root and ros users) is ready to ros.
RUN for user in /root /home/ros; do \
# Source ros and the workspace.
echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then . /opt/ros/${ROS_DISTRO}/setup.bash; fi" \
>> ${user}/.bashrc; \
echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then . ${WORKSPACE}/install/setup.bash; fi" \
>> ${user}/.bashrc; \
echo "alias sc='. ${WORKSPACE}/install/setup.bash'" >> ${user}/.bashrc; \
echo ". /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ${user}/.bashrc; \
done


ENV PYTHONPATH ${WORKSPACE}
COPY src ${WORKSPACE}/src
COPY docker/ros2_command.sh ${WORKSPACE}/ros2_command.sh
COPY config ${WORKSPACE}/config

# Build the source code
RUN cd ${WORKSPACE} && . /opt/ros/humble/setup.sh && \
colcon build --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV RMW_FASTRTPS_USE_QOS_FROM_XML=1
ENV QT_X11_NO_MITSHM=1
# needed, but not defaults
ENV ROS_DOMAIN_ID=111
ENV FASTRTPS_DEFAULT_PROFILES_FILE=
WORKDIR ${WORKSPACE}
ENTRYPOINT [ "/workspace/ros2_command.sh" ]
