# pioneer_ws
## Setup
### Set ROBOT_ID
Please set ROBOT_ID as environment variable.
``` 
$ export ROBOT_ID=pN
```

Ex.)
```
$ export ROBOT_ID=p2
```

Recommend adding to .bashrc
```
$ echo "export ROBOT_ID=p2" >> ~/.bashrc
```

### Install GPS and IMU drivers
``` 
$ sudo chmod +x install.sh
$ ./install.sh
```
After running above, please enable I2C by raspi-config.
```
$ sudo raspi-config
```


## Usage
### Joy Stick
```
LY: ↕  
RX: ↔   
LB: enable
```

## Message
```
Role:       msg_name                    Type
cmd_vel:    /{ROBOT_ID}/cmd_vel         geometry_msgs.msg.Twist
ch_val:     /{ROBOT_ID}/ch_val          std_msgs.msg.Int32MultiArray
gps1:       /{ROBOT_ID}/gps1            sensor_msgs.msg.NavSatFix
quaternion: /{ROBOT_ID}/imu/quaternion  geometry_msgs.msg.Quaternion
eulerAngle: /{ROBOT_ID}/imu/eulerAngle  std_msgs.msg.Float32MultiArray
calibInfo:  /{ROBOT_ID}/imu/calibInfo   std_msgs.msg.Int16MultiArray
```

## Structure
```
Representation:[package_name/run_name] (massage)

[joy_core/joy_node]
  ↓ (joy)
[teleop_core/rover2_joy]  → (enable) / Not used
__↓_(cmd_vel)____________with_Joypad__
[locomotion_core/movebase_kinematics]
  ↓ (ch_val)
[locomotion_core/cmd_roboteq]

[gps_core/run_gps] → (gps1) → To navigation core running on basestation

[imu_core/run_imu] -> (quaternion),(eulerAngle),(calibInfo) → To navigation core running on basestation
```
