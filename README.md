# ROS2 Rotary Encoder Odometry

A project used for taking raw rotary encoder values and mapping it to 1d odometry.

## Including as a Subrepository (Subrepo)

To include this package in a parent ROS2 workspace:

Add as a submodule:

```bash
cd ~/<your_workspace>/src
git submodule add <this-repo-url>  rotary_encoder_odometry
```

Install dependencies:

```bash
cd ~/<your_workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the parent workspace:

```bash
cd ~/<your_workspace>
colcon build --symlink-install
source install/setup.bash
```

## Usage

Run the publisher node:

```bash
ros2 run rotary_encoder_odometry publisher
```

Subscribe to the serial data:

```bash
ros2 topic echo /odom
```

### Parameters

To add parameters when running such as for the distance per tick:

```bash
ros2 run rotary_encoder_odometry publisher --ros-args -p distance_per_tick_m:=0.01
```

To reverse the direction change distance_per_tick_m to a negative value:

```bash
ros2 run rotary_encoder_odometry publisher --ros-args -p distance_per_tick_m:=-0.01
```

## License

This project is licensed under the Apache License 2.0.

## Maintainers

[Andrew Johnson](https://github.com/anjrew) – Maintainer – andrewmjohnson549@gmail.com

## Contributing

Contributions are welcome via PR!

## Other Resources

- Can be paired with [this node](https://github.com/RobotX-Workshops/ros2-serial-reader) for getting values from a serial port