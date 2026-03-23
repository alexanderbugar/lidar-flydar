# wires

s3 devkitc-1 on the base, ld06 and the imu on the head. the motor turns
the head and the lidar still spins on its own. number the slip ring
conductors yourself with a meter. the colors on mine are just mine.

```text
 ring   mine      base                 head
  1     red       regulated 5V         ld06 VCC, 3v3 regulator in
  2     black     GND                  lidar + regulator GND
  3     yellow    GPIO18 / UART1 RX    ld06 TX (3.3V logic)
  4     blue      GPIO8 / SDA          imu SDA
  5     green     GPIO9 / SCL          imu SCL
```

uart is rx only at 230400 8n1. keep the stock lidar motor wiring as it
came. check the pinout for your lidar harness before feeding it 5v.

i2c is 100 khz. use 3.3v pull-ups and count the ones already on the
breakout. no 5v pull-ups on the esp. keep the jumpers short.

qmc5883l joins the same pair on the head, address 0x0d. the gy-271
listing said hmc; actually scan the bus. for a mag capture set
`dumpMag = true` and save the lines starting with `mag,`.

fixed side PWM is GPIO5/6/7 to IN1/2/3, GPIO10 to EN. motor phases go to
the mini, never through the ring. the mini uses the 12V rail with a shared
ground. leave `driveMotor = false` until the head can turn freely.

as5600 stays on the base: GPIO11 SDA, GPIO12 SCL, 3.3V and GND. it's
on Wire1 at 400 khz so a bad ring contact doesn't stop commutation.
check the magnet status before enabling the motor. use a diametric magnet.

zero the axis with the scan plane flat. read `# axis` with sign +1 and
zero 0 first, then put that number in `axisZeroRad`. increasing angle
should tilt +Y toward +Z looking along the right-handed X axis.
flip `axisSign` if it goes the other way.

+X is the shaft, +Z is up when the scan plate is level, +Y finishes the
right-handed frame. align the imu and qmc to those marks on the plate,
with the lidar start ray pointing toward +X. if your lidar reports angles
clockwise from that ray as seen from +Z, set the angle sign accordingly
before using the imu. the stock desk setup only uses the encoder.

if crc climbs with the shaft moving, check the ground and signal
conductors end to end with the power off. for a live test, bridge the
ring with short leads while the head is parked. a rising `ovf` is the
esp dropping bytes and not necessarily a bad ring.
shorten the ground return before blaming the baud rate.

icm-42688-p replaces the mpu on the same head wires. AD0 low gives
0x68, CS pulled high for I2C. both breakouts use 3.3V. build the
`s3_mpu` env for the old board. don't plug both into the same bus.
