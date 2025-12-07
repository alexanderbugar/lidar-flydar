# lidar-flydar

trying to make a cheap 2d lidar see in 3d by spinning the whole thing.
the ld06 plots in 2d now. the motor is still on the protoboard.

grab the serial csv at 921600 and run
`python tools/plot_polar.py capture.csv`. laptop is the entire UI for now.

for udp set the laptop address in config.h and put `#define WIFI_SSID "…"`
and `#define WIFI_PASSWORD "…"` in firmware/include/wifi_local.h.

viewer: install tools/requirements.txt and run `python tools/viewer.py`.
the serial plot is still usefull when the laptop is on the wrong wifi.
