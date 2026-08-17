# mavlink-to-cot

Translates from MAVLink to COT.

## Features

- `egm96` - Enables the EGM96 model for correcting the height from MAVLink
- `egm2008` - Enables the EGM2008 model for correcting the height from MAVLink

By default the `egm2008` feature is selected. You may only use one of the `egm*` features at once.

## MAVLink Requirements

The following MAVLink messages must be provided:
- `GLOBAL_POSITION_INT` is used to source the majority of the position information and we will always wait for the most recent message here before generating a new output. Recommended to receive this at least at twice the target rate you want to emit COT messages.
- `SYSTEM_TIME` is required at least once to establish the offset between the time since boot on the MAVLink vehicle and the unix timestamp. Recommended to maintain at least 0.5Hz on this just to adapt to clock drift and jitter.

Optional MAVLink messages:
- `GPS_RAW_INT` - is used to source a position uncertainty estimate. Must be provided as MAVLink2 and have the `h_acc` and `v_acc` extension fields present to be useful. Recommended to match the `GLOBAL_POSITION_INT` update rate.

## Setup

A configuration file is required,or values can be provided by enviorment variables. An example configuration file with comments can be found within the `example` folder.

Configuration is loaded in order from (with the last one winning):

1. `/etc/mavlink-cot.toml`
1. `config.toml` in the current working directory
1. Enviorment variable prefixed with `MAVLINK_TO_COT`. An example would be `MAVLINK_TO_COT_VEHICLE_UID=FOO`. This would set the vehicle.uid variable.

## Notes

`mavlink-to-cot` is assumed to be running on a system that does not have a reliable clock, such as a onboard computer without RTC. Timestamps are sourced from the vehicle understanding of what unix time is. Until the vehicle has acquired a GPS fix this typically is a free running counter that does a large step to actual unix time once known. Rather then memorize the first offset seen `mavlink-to-cot` will always ingest the most recently received timestamp and use that for message correction.

Because accuracy information on position estimate isn't wildly available in the commonly implemented MAVLink set we combine the GPS estimate with the vehicles fused position estimate. There will obviously be an impact on positional accuracy due to the fusing process, so it can be scaled by a constant scalar to try and make this more accurate. (`UTM_GLOBAL_POSITION` does provide all the information required in a single message, however that requires a more recent version of ArduPilot then this currently tries to support).

[Landlock](https://docs.kernel.org/userspace-api/landlock.html) is used to provide a sandbox when running on Linux. Currently targeting ABI v9 with graceful degradation.
