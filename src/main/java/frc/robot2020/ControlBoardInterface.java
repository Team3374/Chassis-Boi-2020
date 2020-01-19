package frc.robot2020;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getRightTrigger();

    double getLeftTrigger();

    double getTurn();

}
