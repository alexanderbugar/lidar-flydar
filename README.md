# lidar-flydar

ld06 bolted to a 2804 gimbal motor. the ld06 spins a flat scan, and the
motor turns that plane about a second axis. the s3 on the base reads the
shaft angle and sends a rolling cloud over wifi. kind of a 3d lidar, if
you can wait for it to come around.

it works on the desk. turn the head for half a revolution and the walls,
floor and ceiling end up in more or less the right place. pick the base up
and the same wall shows up twice. the imu gives me tilt most of the time
and the mag starts making things up near the motor. neither gives me
usable position as i walk around. the icm helped with noise on the bench
but didn't make this a handheld scanner. i think i'm done trying to tune
my way out of it. next try is a camera and vio, with timestamps i can
actually line up.

example serial output from a desk sweep:

```text
# axis -0.734 rad
# rx crc=2 fields=0 uart=0 ovf=0 qdrop=0
# xyz 1.342 -0.664 0.603
```

a clean sweep at 0.75 rad/s takes about 4.2 seconds for half a turn.
call it 18k returns if most of the rays hit something. chair legs still
get missed. the lidar origin is treated as the shaft center, which is only
as good as my drill holes.

## get a scan

1. wire it up from [wiring](docs/wiring.md) and [power](docs/power.md).
   the [parts list](hardware/bom.md) has the plastic notes. photos coming.
2. set the laptop's IP as `udpHost` in
   [config.h](firmware/include/config.h). make `firmware/include/wifi_local.h`
   with `#define WIFI_SSID "your ssid"` and `#define WIFI_PASSWORD "your password"`.
   the file is ignored. use an Arduino 3.x-capable espressif32 platform.
3. `pio run -e s3 -t upload`, then `pio device monitor -b 921600` for the
   encoder zero and rx counters. for the mpu6050 instead of the icm, use
   `s3_mpu`. start with `driveMotor` off and turn by hand. set the
   mechanical zero and shaft sign before enabling the drive.
4. in a Python 3.11 or 3.12 environment,
   `python -m pip install -r tools/requirements.txt`, then
   `python tools/viewer.py`. UDP goes to port 4210.

```sh
python tools/viewer.py --record desk.fdl
python tools/export_ply.py desk.fdl desk.ply --voxel 0.025
python tools/replay.py desk.fdl --speed 0.5
pio test -e native
```

play the capture into a running viewer. the record option uses a new file
each time. point times in FLY1 are the esp's microsecond clock plus a
per-return offset. the FDL1 log also keeps laptop arrival times so replay
can keep the spacing. sequence gaps show missed datagrams. the packet's
drop counter is cumulative and counts firmware queue and send failures.

the old 2d plot still takes a serial capture. set `serialPolar = true`,
save the lines and run `python tools/plot_polar.py capture.csv`.
for magnetic calibration, stop the drive, set `dumpMag = true`, turn the
whole head in different directions and run `python tools/mag_cal.py capture.csv`.
paste the two initializers into config.h before setting `magCalibrated`.
leave `stabilize` off for a fixed base.

ota is optional and only runs in the motor-off build. define
`OTA_PASSWORD` in wifi_local.h and flash over usb once. for later uploads
set `FLYDAR_OTA_PASSWORD` in the shell and run `pio run -e s3_ota -t upload`
on the same network. keep usb available. see the [desk notes](docs/devlog.md).
