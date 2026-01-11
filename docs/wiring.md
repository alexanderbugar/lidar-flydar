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
