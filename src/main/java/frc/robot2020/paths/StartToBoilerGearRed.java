package frc.robot2020.paths;

import frc.robot2020.paths.profiles.PathAdapter;
import frc.util.control.Path;
import frc.util.math.RigidTransform2d;

/**
 * Path from the red alliance wall to the red boiler peg.
 * 
 * Used in GearThenHopperShootModeRed
 * 
 * @see GearThenHopperShootModeRed
 * @see PathContainer
 */
public class StartToBoilerGearRed implements PathContainer {

    @Override
    public Path buildPath() {
        return PathAdapter.getRedGearPath();
    }

    @Override
    public RigidTransform2d getStartPose() {
        return PathAdapter.getRedStartPose();
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}