package frc.robot2020.auto.modes;

import frc.robot2020.auto.AutoModeBase;
import frc.robot2020.auto.AutoModeEndedException;

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot standstill
 */
public class StandStillMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        //System.out.println("Starting Stand Still Mode... Done!");
    }
}