# Playground to test node communication scenarios with FastDDS middleware


# Notes
TCP setup is complicated because it differentiates between server and client. N

https://docs.ros.org/en/humble/Tutorials/Advanced/FastDDS-Configuration.html


# on particpants
Selecting a Participant Profile at Runtime
When launching your ROS 2 nodes, you can specify which participant profile to use by setting environment variables or through programmatic configurations in custom DDS applications. In ROS 2, however, direct support for selecting DDS profiles at the node level is limited by the ROS 2 middleware abstraction.

For applications that directly use Fast DDS, you can select a profile like so:

Environment Variable: Set an environment variable to specify the default profile to be used by participants. This approach affects all DDS participants created in the process where the environment variable is set.

API Call: In custom Fast DDS applications, you can select a specific participant profile programmatically using Fast DDS APIs by specifying the profile name when creating the participant.

Limitations in ROS 2
In ROS 2, the middleware abstraction layer (rmw) simplifies the interaction with the underlying DDS layer, which means that fine-grained control over DDS features (like selecting specific participant profiles for individual nodes within a single process) is not directly exposed through standard ROS 2 interfaces. The rmw implementation for Fast DDS will typically use environment variables or a single XML configuration file to apply settings globally to all nodes within a process.

Conclusion
While an XML configuration file for Fast DDS can contain definitions for multiple participant profiles, applying these profiles selectively to different ROS 2 nodes within the same process is not straightforward due to ROS 2's design. The configuration applied by the XML file is generally intended to be process-wide, affecting all nodes running in that process.



https://fast-dds.docs.eprosima.com/en/v2.11.1/docker/shm_docker.html

I am missing the automatic fallback. E.g. if shared memory is not host=ipc'ed, its still not falling back to udp
<useBuiltinTransports>true</useBuiltinTransports> actually makes things worse, because it I guess wants to use shared memory, but ill configured (e.g. semgent size to small)

If the particupant is set to to use shared memory, it is better configured correcty. Sizes,user permissions, ipc=host hashign

https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/datasharing.html#datasharing-delivery


docker compose  -f ./pub_udp_sub_udp.compose up --build
sudo tshark -i any -Y "rtps" -q -z io,stat,1,"rtps"

ros2 run rviz2 rviz2 -d config/default.rviz
