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
export DINGO_PACS_ENABLED=0
export DINGO_URDF_EXTRAS=empty.urdf
```
###### Launch
```bash
export DINGO_MOTOR_PARAMS='base'
export DINGO_MAG_CONFIG=$(catkin_find dingo_base config/mag_config_default.yaml --first-only)
```

## 2D Laser Scan
You can add two lidar scans and select between the `lms1xx` and `ust10`.

#### Primary Laser
```bash
export DINGO_LASER=0
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
export DINGO_LASER_SECONDARY=0
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
export DINGO_LASER_3D=0
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
export DINGO_REALSENSE=0
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
export DINGO_IMU_MICROSTRAIN=0
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
