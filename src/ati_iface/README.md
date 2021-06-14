ATI Force/Torque sensor interface
=================================
This project provides an OROCOS component for reading and [ATI F/T sensor](https://www.ati-ia.com/products/ft/sensors.aspx) with [Beckhoff EtherCAT](https://www.beckhoff.com/EtherCAT/).

NOTE: If you want to use an ATI F/T sensor with EtherCAT interface (factory supplied), refer to [rtt_soem_ati](https://gitlab.kuleuven.be/ras-hardware/rtt_soem_ati.git).

Dependencies
------------
Necessary software to run these applications<sup>1</sup> :
-   [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
-   [ROS Kinetic](http://wiki.ros.org/kinetic)
-   OROCOS Toolchain 2.9: You can either [build from source](https://github.com/orocos/rtt_ros_integration/tree/toolchain-2.9) or use command line<sup>2</sup> :
    ```bash
    sudo apt-get install ros-kinetic-rtt-ros-integration
    ```

-   [SOEM for ROS](https://github.com/mgruhler/soem):
    ```bash
    sudo apt-get install ros-kinetic-soem
    ```

-   [rtt_soem](https://github.com/orocos/rtt_soem): An OROCOS wrapper for SOEM. To be installed next to the *ati_iface* directory in the *src* of your *catkin_ws*. The OROCOS Deployer needs rights to configure *soem_master*:
    ```bash
    sudo setcap cap_net_raw+ep /opt/ros/kinetic/bin/deployer-gnulinux
    ```

Building
--------
After building...
``` bash
cd ~/catkin_ws
catkin_make
```
... your workspace should look like this:
```
catkin_ws
├── src
|    ├── rtt_soem
|    ├── ati_iface
|    └── ...
├── build
└── devel

```

Important remarks
-------
<sup>1</sup> Older or newer versions of software might work but this is not tested.

<sup>2</sup> Note that if you install OROCOS Toolchain 2.9 from **binaries (command line)** you might not get the latest release. In that case you could have issues with reading the *soem_beckhoff_drivers* custom messages. To solve this issue, you have to update some *.cmake* files as indicated [here](https://github.com/orocos-toolchain/rtt/pull/244/files).
