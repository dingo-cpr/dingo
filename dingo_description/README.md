# Dingo Description
To facilitate editing payloads and other features of the Dingo, we use the following environment variables. 

Every sensor has three types of environment variables: 
1. **Enable**: these will add the sensor to the URDF and start the launch file. 
2. **Launch**: these correspond to parameters exclusively in the launch file.
3. **Description**: these correspond to parameters excluvesively in the URDF.

## Dingo Variables
```bash
export DINGO_OMNI=0
export M_PI=3.14159265359
```
###### Description
```bash
export DINGO_PACS_ENABLED=1
export DINGO_URDF_EXTRAS=empty.urdf
```
###### Launch
```bash
export DINGO_MOTOR_PARAMS='base'
export DINGO_MAG_CONFIG=$(catkin_find dingo_base config/mag_config_default.yaml --first-only)
```

## PACS
Enabling the PACS system overwrites the standard mounting locations (front_mount, front_a_mount,..., rear_mount, rear_a_mount,...,etc.) with mounting locations C01, C02,..., C06, etc. These mounting locations are at level 0 and the middle row (the only area of the Dingo that has mounting holes).

A riser (an entire plate with a 5x6 grid of mounting locations) can be added using the `DINGO_PACS_RISER` variable. Specify the height at which the riser will be added. For example, adding a PACS riser at level 1 will add a riser with 10 cm standoffs: `export DINGO_PACS_RISER=1`. When a PACS riser is added at level 1, a grid of mounting locations named A11, A12,..., B11, B12,...,E16. Notice that the middle character in the naming scheme corresponds to the height. 

Adding a bracket at any location is also done via environment variables. Brackets are thin plate adapters that allow a wide selection of sensors to be added to the existing grid. To add a bracket to the A11 location, use `DINGO_A11_BRACKET_ENABLED=1`. 
```bash
# Enable PACS
export DINGO_PACS_ENABLED=1
# Add Riser at with 10 cm Height
export DINGO_PACS_RISER=1 # Adds riser at Level 1 (i.e. 10 cm), each level adds another 10 cm
# Add Bracket at A11 Location
export DINGO_A11_BRACKET_ENABLED=1
export DINGO_A11_BRACKET_TYPE=horizontal # or horizontal_large or vertical
export DINGO_A11_BRACKET_XYZ="0 0 0"
export DINGO_A11_BRACKET_RPY="0 0 0"
export DINGO_A11_BRACKET_EXTENSION="0" # distance from surface of plate/riser to surface of bracket
```

## 2D Laser Scan
You can add two lidar scans and select between the `lms1xx` and `ust10`.

#### Primary Laser
```bash
export DINGO_LASER=1
export DINGO_LASER_MODEL='lms1xx' # or 'ust10'
```
###### Launch
```bash
export DINGO_LASER_HOST='192.168.131.20'
export DINGO_LASER_TOPIC='front/scan'
```
###### Description
```bash
export DINGO_LASER_MOUNT='front'
export DINGO_LASER_TOWER=1
export DINGO_LASER_PREFIX=${DINGO_LASER_MOUNT}
export DINGO_LASER_PARENT=${DINGO_LASER_MOUNT}_mount
export DINGO_LASER_OFFSET='0 0 0'
export DINGO_LASER_RPY='0 0 0'
```
> By default, the `PARENT` link and the `PREFIX` of links are determined by the value given to `MOUNT`. However, these can be set independently for further customization options. 


#### Secondary Laser:
```bash
export DINGO_LASER_SECONDARY=1
export DINGO_LASER_SECONDARY_MODEL='lms1xx' # or 'ust10'
```
###### Launch
```bash

export DINGO_LASER_SECONDARY_HOST='192.168.131.20'
export DINGO_LASER_SECONDARY_TOPIC='rear/scan'
```
###### Description
```bash
export DINGO_LASER_SECONDARY_MOUNT='rear'
export DINGO_LASER_SECONDARY_TOWER=1
export DINGO_LASER_SECONDARY_PREFIX=${DINGO_LASER_SECONDARY_MOUNT}
export DINGO_LASER_SECONDARY_PARENT=${DINGO_LASER_SECONDARY_MOUNT}_mount
export DINGO_LASER_SECONDARY_OFFSET='0 0 ${M_PI}'
export DINGO_LASER_SECONDARY_RPY='0 0 ${M_PI}'
```

## 3D Laser
```bash
export DINGO_LASER_3D=1
export DINGO_LASER_3D_MODEL='vlp16'
```
###### Launch
```bash
export DINGO_LASER_3D_HOST='192.168.131.20'
export DINGO_LASER_3D_TOPIC='front/points'
```
###### Description
```bash
export DINGO_LASER_3D_MOUNT='front'
export DINGO_LASER_3D_TOWER=1
export DINGO_LASER_3D_ANGLE=0
export DINGO_LASER_3D_PREFIX=${DINGO_LASER_3D_MOUNT}
export DINGO_LASER_3D_PARENT=${DINGO_LASER_3D_MOUNT}_mount
export DINGO_LASER_3D_OFFSET='0 0 0'
export DINGO_LASER_3D_RPY='0 0 0'
```

## Realsense
```bash
export DINGO_REALSENSE=1
export DINGO_REALSENSE_MODEL='d435' # or 'd435i', 'd415', 'd455', 'l515'
```
###### Launch
```bash
export DINGO_REALSENSE_TOPIC='realsense'
```
###### Description
```bash
export DINGO_REALSENSE_MOUNT='front'
export DINGO_REALSENSE_OFFSET='0 0 0'
export DINGO_REALSENSE_RPY='0 0 0'
```

## Microstrain IMU
```bash
export DINGO_IMU_MICROSTRAIN=1
```
###### Launch
```bash
export DINGO_IMU_MICROSTRAIN_NAME='microstrain'
export DINGO_IMU_MICROSTRAIN_PORT='/dev/microstrain'
```
###### Description
```bash
export DINGO_IMU_MICROSTRAIN_LINK='microstrain_link'
export DINGO_IMU_MICROSTRAIN_PARENT='imu_link'
export DINGO_IMU_MICROSTRAIN_OFFSET='0 0 0'
export DINGO_IMU_MICROSTRAIN_RPY='0 0 0'
```
