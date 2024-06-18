# Unitree Go1 navigation package

A package with everything you need to run traversability algorithms

## To use the cameras script, install `sshpass`:
```
sudo apt install sshpass
```

## Setup

### If using a serial GPS, set the permission with:
```
sudo adduser $USER $(stat --format="%G" /dev/ttyACM0 )
```

### To use dLIO as the state estimator

clone the DLIO package

In `dlio/slam`, change the extrinsics to:
```
imu:
    calibration: true
    intrinsics:
      accel:
        bias: [ 0.0, 0.0, 0.0 ]
        sm:   [ 1.,  0.,  0.,
                0.,  1.,  0.,
                0.,  0.,  1. ]
      gyro:
        bias: [ 0.0, 0.0, 0.0 ]

  extrinsics:
    baselink2imu:
      t: [ 0., 0., 0. ]
      R: [ 1.,  0.,  0.,
           0.,  1.,  0.,
           0.,  0.,  1. ]
    baselink2lidar:
      t: [ -0.055,  0.,  0.230 ]
      R: [ 1.,  0.,  0.,
           0.,  1.,  0.,
           0.,  0.,  1. ]
```
