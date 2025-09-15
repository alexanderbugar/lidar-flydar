# lidar-flydar

trying to make a cheap 2d lidar see in 3d by spinning the whole thing.
right now the esp blinks. solid start.

grab the serial csv at 921600 and run
`python tools/plot_polar.py capture.csv`. laptop is the entire UI for now.

for udp set the laptop address in config.h and put `#define WIFI_SSID "…"`
and `#define WIFI_PASSWORD "…"` in firmware/include/wifi_local.h.
