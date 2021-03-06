package frc.robot2020.paths;

import frc.util.control.Path;
import frc.util.math.RigidTransform2d;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    Path buildPath();

    RigidTransform2d getStartPose();

    boolean isReversed();
}
