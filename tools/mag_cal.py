"""Fit an ellipsoid to the serial mag,x,y,z lines."""
import argparse
import csv

import numpy as np


def read_samples(path):
    with open(path, newline="") as stream:
        for row in csv.reader(stream):
            if len(row) != 4 or row[0] != "mag":
                continue
            try:
                xyz = tuple(float(value) for value in row[1:])
            except ValueError:
                continue
            if np.isfinite(xyz).all():
                yield xyz


def fit(samples):
    if len(samples) < 40:
        raise ValueError("need at least 40 points from different orientations")

    # center and scale before solving; counts squared are huge next to counts.
    origin = samples.mean(axis=0)
    delta = samples - origin
    scale = np.sqrt(np.mean(np.sum(delta**2, axis=1)))
    if scale <= 0:
        raise ValueError("samples do not cover an ellipsoid")

    x, y, z = (delta / scale).T
    design = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, x, y, z))
    coeff, _, rank, singular = np.linalg.lstsq(design, np.ones(len(samples)), rcond=None)
    if rank != 9 or singular[-1] < 0.015 * singular[0]:
        raise ValueError("turn it over too; there is almost no data on one axis")

    xx, yy, zz, xy, xz, yz = coeff[:6]
    shape = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    eigenvalues, axes = np.linalg.eigh(shape)
    if np.any(eigenvalues <= 0):
        raise ValueError("fit is not an ellipsoid; move away from steel and retry")

    center = np.linalg.solve(shape, -0.5 * coeff[6:])
    radius2 = 1 + center @ shape @ center
    bias = origin + scale * center
    correction = ((axes * np.sqrt(eigenvalues / radius2)) @ axes.T) / scale
    length = np.linalg.norm((samples - bias) @ correction.T, axis=1)
    rms = np.sqrt(np.mean((length - 1)**2))
    if rms > 0.15:
        raise ValueError(f"field isn't stable enough (radial rms {rms:.3f})")
    return bias, correction, rms


def initializer(row):
    return "{" + ", ".join(f"{value:.8e}f" for value in row) + "}"


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("capture")
    args = cli.parse_args()
    samples = np.array(list(read_samples(args.capture)))
    try:
        bias, correction, rms = fit(samples)
    except (ValueError, np.linalg.LinAlgError) as exc:
        cli.error(str(exc))
    print(f"// {len(samples)} samples, radial rms {rms:.3f}")
    print("constexpr float magBias[3] = " + initializer(bias) + ";")
    matrix = ",\n    ".join(initializer(row) for row in correction)
    print("constexpr float magMatrix[3][3] = {\n    " + matrix + "\n};")


if __name__ == "__main__":
    main()
