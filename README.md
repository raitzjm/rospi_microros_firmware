# rospi_microros_firmware


To get started, clone this repo into your workspace.

Then, go to this repo's folder:

```bash
cd rospi_microros_firmware
```

Then clone the microROS Zephyr module:
```bash
git clone -b humble https://github.com/micro-ROS/micro_ros_zephyr_module.git
```

Or my fork if I pushed a new patch that is not available in the upstream yet: 
```bash
git clone -b humble https://github.com/raitzjm/micro_ros_zephyr_module
```

Since the microROS Zephyr repo is not in a Zephyr module format, run the following commands to set it up as a module on this project:
```bash
mv microros_zephyr_module/modules . \
rm -r microros_zephyr_module
```

