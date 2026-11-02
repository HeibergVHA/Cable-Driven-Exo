# P5 Cable-Driven-Exo



## Current docker ros2 workflow
cd ---->>> delta_ws

docker compose build dev

docker compose run --rm dev

docker compose exec dev bash

colcon build --symlink-install
// If first time build, restart container and it will source by itself. Otherwise:
source install/setup.bash

ros2 launch python_i2c_scanner abc.launch.py
ros2 launch delta_motors motors.launch.py
ros2 run delta_motors motor_cmd
motor> 1 10          # motor 1 to 10°
motor> 1 0 20        # back to 0° at 20 r/min
motor> step 1 5      # relative +5°
motor> get
motor> stop          # emergency stop

ros2 run cpp_package1232 AS5600_reader



## General docker compose: ##

docker compose build        # build the image
docker compose up           # start the container(s) in the foreground
docker compose up -d        # start in the background
docker compose logs -f      # watch output
docker compose down         # stop and remove
docker compose run --rm ros bash   # interactive shell, like docker run -it --rm




## can setup on PI

delta@DELTA:~ $ ip link show can0
16: can0: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can 
delta@DELTA:~ $ sudo ip link set can0 up type can bitrate 1000000
