"""Plot a serial capture: angle_rad,mm,x_mm,y_mm,intensity."""
import argparse
import csv

import matplotlib.pyplot as plt


def read_capture(path):
    with open(path, newline="") as stream:
        for row in csv.reader(stream):
            if len(row) != 5:
                continue
            try:
                angle, distance = map(float, row[:2])
            except ValueError:
                continue
            if distance > 0:
                yield angle, distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture")
    args = parser.parse_args()
    points = list(read_capture(args.capture))
    if not points:
        parser.error("no points in capture")
    angle, distance = zip(*points)
    ax = plt.subplot(projection="polar")
    ax.scatter(angle, distance, s=2)
    ax.set_theta_direction(-1)
    plt.show()


if __name__ == "__main__":
    main()
