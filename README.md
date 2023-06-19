# perception_ecu_launch

A launch configuration repository for [tier4/perception_ecu_container](https://github.com/tier4/perception_ecu_container).

## Setting up camera processing mode

In `config/camera_modes.yaml`, specify the camera processing mode.
By default, only one camera is activated as `object_recognition` mode.

```yaml
camera0: object_recognition
camera1: disable
camera2: disable
camera3: disable
camera4: disable
camera5: disable
camera6: disable
camera7: disable
```

To change the mode of camera processing, prepare your file `camera_modes.yaml`.
Here is an example of starting two cameras in `object_recognition` mode and two cameras in `image_capture` mode.

```yaml
camera0: object_recognition
camera1: object_recognition
camera2: image_capture
camera3: image_capture
camera4: disable
camera5: disable
camera6: disable
camera7: disable
```

Currently, the following modes are supported. Additional modes will be added in the future.

| Mode               | Description                                 |
| ------------------ | ------------------------------------------- |
| disable            | Camera will not be activated                |
| image_capture      | Activate only a camera driver               |
| object_recognition | Activate camera driver & object recognition |

Device ports, calibration parameters, etc. required to activate each camera are defined in `individual_params`.
Please refer to [perception_ecu_individual_params](https://github.com/tier4/perception_ecu_individual_params).

## Launch camera processing

The following commands start up your camera processing.

```sh
ros2 launch perception_ecu_launch perception_ecu.launch.py
```

If you define `camera_modes.yaml` by yourself, you can launch your camera processing by the following commands.

```sh
ros2 launch perception_ecu_launch perception_ecu.launch.py \
    camera_modes_path:=<your-path>/camera_modes.yaml
```

For cameras that support FSYNC input, the shutter trigger can be input by using [tier4/sensor_trigger](https://github.com/tier4/sensor_trigger) package and by the following launch option.

```sh
ros2 launch perception_ecu_launch perception_ecu.launch.py \
    launch_sensor_trigger:=True
```

For [TIER IV Automotive HDR Camera C1](https://sensor.tier4.jp/automotive-hdr-camera), slave mode setting is required.
Please refer to [tier4/tier4_automotive_hdr_camera](https://github.com/tier4/tier4_automotive_hdr_camera).

## Related Links
- [tier4/perception_ecu_container](https://github.com/tier4/perception_ecu_container)
- [tier4/perception_ecu_individual_params](https://github.com/tier4/perception_ecu_individual_params)
