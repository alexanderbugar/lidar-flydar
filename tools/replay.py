"""Replay an FDL1 record at the original pacing, give or take the OS."""
import argparse
import math
import socket
import time

from wire import records


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("capture")
    cli.add_argument("--host", default="127.0.0.1")
    cli.add_argument("--port", type=int, default=4210)
    cli.add_argument("--speed", type=float, default=1)
    args = cli.parse_args()
    if not math.isfinite(args.speed) or args.speed <= 0:
        cli.error("speed must be finite and positive")

    try:
        with open(args.capture, "rb") as stream, socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        ) as sock:
            first_us = None
            start_s = time.monotonic()
            for elapsed_us, data in records(stream):
                if first_us is None:
                    first_us = elapsed_us
                target = start_s + (elapsed_us - first_us) / (1e6 * args.speed)
                while (left := target - time.monotonic()) > 0:
                    time.sleep(min(left, 0.1))
                sock.sendto(data, (args.host, args.port))
    except KeyboardInterrupt:
        return
    except (OSError, ValueError) as exc:
        cli.error(str(exc))


if __name__ == "__main__":
    main()
