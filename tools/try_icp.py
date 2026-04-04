"""See if two separate desk captures line up."""
import argparse
import math

import numpy as np
import open3d as o3d


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("moving")
    cli.add_argument("fixed")
    cli.add_argument("--voxel", type=float, default=0.035)
    cli.add_argument("--match", type=float, default=0.14)
    args = cli.parse_args()
    if not all(math.isfinite(n) and n > 0 for n in (args.voxel, args.match)):
        cli.error("voxel and match distances must be positive")

    clouds = [o3d.io.read_point_cloud(path).voxel_down_sample(args.voxel)
              for path in (args.moving, args.fixed)]
    if any(len(cloud.points) < 20 for cloud in clouds):
        cli.error("each cloud needs at least 20 points after downsampling")
    moving, fixed = clouds
    icp = o3d.pipelines.registration
    result = icp.registration_icp(
        moving, fixed, args.match, np.eye(4),
        icp.TransformationEstimationPointToPoint(),
        icp.ICPConvergenceCriteria(max_iteration=70),
    )
    print(result.transformation)
    print(f"fitness {result.fitness:.3f}, rms {result.inlier_rmse:.3f} m")


if __name__ == "__main__":
    main()
