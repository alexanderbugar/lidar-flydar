# bits

| part | notes |
| --- | --- |
| esp32-s3 devkitc-1 | n8, usb-c board |
| ldrobot ld06 | ld19 should speak the same packet; mine is an ld06 |
| 2804 gimbal bldc | mine is seven pole pairs, count yours |
| simplefoc mini v1.1 | drv8313, 12 v motor side |
| hollow-shaft slip ring | need five conductors, count continuity before soldering |
| icm-42688-p breakout | on the head, replaced the mpu6050 |
| qmc5883l breakout | also on the head, away from the motor if possble |
| as5600 and diametric magnet | encoder on base, magnet on shaft |
| 12 v brick and 5 v buck | common ground, logic on its own branch |
| screws and standoffs | nylon near the mag |

## plastic

split clamp for the stationary half of the ring, two M3 heat-set inserts for
the ld06 plate. leave a loop where the cable exits the shaft. v2 had a nice
sharp corner precisely where the wires wanted to go.

todo: mount_v3.stl not committed yet. check the magnet gap with the cover on.
