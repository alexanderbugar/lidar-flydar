# power

one 12 V / 2 A brick, split at the input. one leg to the mini, the
other to a 5 V / 2 A buck for the s3 and the slip ring. join grounds
at the split and run the motor return straight there. don't route it
through the devkit header or the ring.

| 5 V load | budget |
| --- | ---: |
| s3 devkit with wifi bursts | 450 mA |
| ld06 with its own motor turning | 350 mA |
| head 3.3 V regulator and sensors | 50 mA |
| margin for startup and wiring | 350 mA |
| total to allow | 1.2 A |

motor side budget is roughly 0.8 A at startup on the 12 V rail. these
are sizing numbers. the meter on the desk misses short peaks.

the s3 used to reboot just as the head started. the bench supply's long
clip leads weren't doing it any favors. put 470 uF / 25 V at VM/GND on
the mini and 470 uF / 10 V across 5 V/GND at the base buck output. mind
the cap polarity. use short leads and leave the ceramics on the boards.
check that the buck still starts cleanly into the extra capacitance.

usb for flashing: unplug the buck feed to the devkit before attaching
the usb cable, or use a data-only power-isolated setup. the head still
needs 5 V if you want data. the mini's onboard 3.3 V isn't for powering
the head.

the firmware waits 600 ms before alignment and takes just under two
seconds to ramp to 0.75 rad/s after it. the align voltage still has to
move the rotor. if it doesn't, check the phases and pole pairs before
turning the voltage up. add a 10k pull-down at EN if your driver board
doesn't already have one.
