package frc.robot2020.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.ctre.TalonSRX.StatusFrameRate;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.VelocityPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.TalonSRX.VelocityMeasurementPeriod;

import frc.robot2020.Constants;
import frc.robot2020.Kinematics;
//import frc.robot2020.RobotState;
//import frc.robot2020.ShooterAimingParameters;
import frc.robot2020.loops.Loop;
import frc.robot2020.loops.Looper;
import frc.util.DriveSignal;
import frc.util.ReflectingCSVWriter;
import frc.util.Util;
import frc.util.control.Lookahead;
import frc.util.control.Path;
import frc.util.control.PathFollower;
import frc.util.drivers.TalonSRXFactory;
import frc.util.drivers.NavX;
import frc.util.math.RigidTransform2d;
import frc.util.math.Rotation2d;
import frc.util.math.Twist2d;

import java.util.Arrays;
import java.util.Optional;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons, one solenoid and 2 pistons to shift gears,
 * and a navX board. The Drive subsystem has several control methods including open loop, velocity control, and position
 * control. The Drive subsystem also has several methods that handle automatic aiming, autonomous path driving, and
 * manual control.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    //private static final int kLowGearPositionControlSlot = 0;

    private static final int kLowGearVelocityControlSlot = 0;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
        PATH_FOLLOWING, // used for autonomous driving
        AIM_TO_GOAL, // turn to face the boiler
        TURN_TO_HEADING, // turn in place
        DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
        DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
    }

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state) {
        if (state == DriveControlState.AIM_TO_GOAL ||
                state == DriveControlState.TURN_TO_HEADING ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH) {
            return true;
        }
        return false;
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final NavX mNavXBoard;

    // Controllers
    //private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;

    // Hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;

    // Logging
    private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
                //setVelocitySetpoint(0, 0);
                mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
                    break;
                case VELOCITY_SETPOINT:
                    break;
                case AIM_TO_GOAL:
                   // if (!Superstructure.getInstance().isShooting()) {
                        //updateGoalHeading(timestamp);
                   // }
                    // fallthrough intended
                    break;
                case TURN_TO_HEADING:
                   // updateTurnToHeading(timestamp);
                    break;
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
    };

    private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
       // mLeftMaster.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
       //changeControlMode doesn't exist anymore
       //ToDo: update mode when set power
       final ErrorCode leftSensorPresent = mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
       if (leftSensorPresent != ErrorCode.OK) {
           DriverStation.reportError("Could not detect left encoder:", false);
       }
        //ToDo: Update with final encoders
        mLeftMaster.setInverted(false);
        mLeftMaster.setSensorPhase(true);
        //pick a value so positive is a positive change in sensor. Change this if movement counts are backwards
        //Change as needed
        mLeftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    
        mLeftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        mLeftMaster.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
            

        mLeftSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveId,
                Constants.kLeftDriveMasterId);
        mLeftSlave.setInverted(false);

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kRightDriveMasterId);
        // mLeftMaster.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
        //changeControlMode doesn't exist anymore
        //ToDo: update mode when set power
        final ErrorCode rightSensorPresent = mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        if (rightSensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect right encoder:", false);
        }
         //ToDo: Update with final encoders
         mRightMaster.setInverted(false);
         mRightMaster.setSensorPhase(true);
         //pick a value so positive is a positive change in sensor. Change this if movement counts are backwards
         //Change as needed
         mRightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);

         mRightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
         mRightMaster.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);


        mRightSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriverSlaveId,
                Constants.kRightDriveMasterId);
        mRightSlave.setInverted(false);

        reloadGains();

        mIsHighGear = false;
        setHighGear(true);
        setOpenLoop(DriveSignal.NEUTRAL);

        // Path Following stuff
        mNavXBoard = new NavX(SPI.Port.kMXP);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv",
                PathFollower.DebugOutput.class);
                mLeftMaster.config_kP(0, Constants.kDriveP);
                mLeftMaster.config_kI(0, Constants.kDriveI);
                mLeftMaster.config_kD(0, Constants.kDriveD);
                mLeftMaster.configMotionCruiseVelocity(Constants.kDriveCruiseVelocity);
                mLeftMaster.configMotionAcceleration(Constants.kDriveAcceleration);

                mRightMaster.config_kP(0, Constants.kDriveP);
                mRightMaster.config_kI(0, Constants.kDriveI);
                mRightMaster.config_kD(0, Constants.kDriveD);
                mRightMaster.configMotionCruiseVelocity(Constants.kDriveCruiseVelocity);
                mRightMaster.configMotionAcceleration(Constants.kDriveAcceleration);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.configNeutralDeadband(0.04, 0);
            mRightMaster.configNeutralDeadband(0.04, 0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master.
        //mRightMaster.set(ControlMode.PercentOutput,-signal.getRight());
        //mLeftMaster.set(ControlMode.PercentOutput,signal.getLeft());
        mRightMaster.set(ControlMode.PercentOutput,-signal.getRight());
        mLeftMaster.set(ControlMode.PercentOutput,signal.getLeft());
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlave.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        final double left_speed = getLeftVelocityInchesPerSec();
        final double right_speed = getRightVelocityInchesPerSec();
        SmartDashboard.putString("Drive State", mDriveControlState.toString());
        SmartDashboard.putNumber("left speed (ips)", left_speed);
        SmartDashboard.putNumber("right speed (ips)", right_speed);
        if (usesTalonVelocityControl(mDriveControlState)) {
           // SmartDashboard.putNumber("left speed error (ips)",
             //       rpmToInchesPerSecond(mLeftMaster.getSetpoint()) - left_speed);
            //SmartDashboard.putNumber("right speed error (ips)",
              //      rpmToInchesPerSecond(mRightMaster.getSetpoint()) - right_speed);
        } else {
            SmartDashboard.putNumber("left speed error (ips)", 0.0);
            SmartDashboard.putNumber("right speed error (ips)", 0.0);
        }
        synchronized (this) {
            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
                SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
                SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
            } else {
                SmartDashboard.putNumber("drive CTE", 0.0);
                SmartDashboard.putNumber("drive ATE", 0.0);
            }
        }      //ToDo: add better debugging
        //SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getSelectedSensorPosition());
        //SmartDashboard.putNumber("right position (rotations)", mRightMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
        SmartDashboard.putNumber("gyro pos", getGyroAngle().getDegrees());
        SmartDashboard.putBoolean("drive on target", isOnTarget());
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
        mNavXBoard.zeroYaw();
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(mDriveControlState)) {
            // We entered a velocity control state.
            mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            setBrakeMode(true);
        }
    }

   
    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */

     //Todo: diagnose inchespersecondtorpm stuff
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(mDriveControlState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
           // mLeftMaster.set(ControlMode.Velocity, inchesPerSecondToRpm(left_inches_per_sec));
            //mRightMaster.set(ControlMode.Velocity, inchesPerSecondToRpm(right_inches_per_sec));
            mLeftMaster.set(ControlMode.Velocity, rotationsToTicks(inchesToRotations(left_inches_per_sec))/10);
            mRightMaster.set(ControlMode.Velocity, rotationsToTicks(inchesToRotations(right_inches_per_sec))/10);
        } else {
            System.out.println("Hit a bad velocity control state");
            mLeftMaster.set(ControlMode.Velocity, 0.0);
            mRightMaster.set(ControlMode.Velocity, 0.0);
            //Todo: check on controlmode.velocity
        }
    }

    /**
     * Adjust position setpoint (if already in position mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
        if (usesTalonPositionControl(mDriveControlState)) {
            mLeftMaster.set(ControlMode.Position, inchesToRotations(left_position_inches));
            mRightMaster.set(ControlMode.Position, inchesToRotations(right_position_inches));
        } else {
            System.out.println("Hit a bad position control state");
            mLeftMaster.set(ControlMode.Position, 0);
            mRightMaster.set(ControlMode.Position, 0);
        }//check on controlmode.position
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }
    private static double rotationsToTicks(double rotations) {
        return (rotations * Constants.kTicksPerRevolution);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(mLeftMaster.getSelectedSensorPosition());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(mRightMaster.getSelectedSensorPosition());
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(mLeftMaster.getSelectedSensorVelocity());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(mRightMaster.getSelectedSensorVelocity());
    }

    public synchronized Rotation2d getGyroAngle() {
        return mNavXBoard.getYaw();
    }

    public synchronized NavX getNavXBoard() {
        return mNavXBoard;
    }

    public synchronized void setGyroAngle(Rotation2d angle) {
        mNavXBoard.reset();
        mNavXBoard.setAngleAdjustment(angle);
    }

    public synchronized double getGyroVelocityDegreesPerSec() {
        return mNavXBoard.getYawRateDegreesPerSec();
    }

    /**
     * Update the heading at which the robot thinks the boiler is.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
 
   
    public synchronized boolean isOnTarget() {
        // return true;
        return mIsOnTarget;
    }

    public synchronized boolean isAutoAiming() {
        return mDriveControlState == DriveControlState.AIM_TO_GOAL;
    }

    
    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    public boolean isApproaching() {
        return mIsApproaching;
    }

    public synchronized boolean isDoneWithTurn() {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING) {
            return mIsOnTarget;
        } else {
            System.out.println("Robot is not in turn to heading mode");
            return false;
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
    }

    public synchronized double getAccelX() {
        return mNavXBoard.getRawAccelX();
    }

    @Override
    public void writeToLog() {
        mCSVWriter.write();
    }

    public boolean checkSystem() {
        System.out.println("Testing DRIVE.---------------------------------");
        final double kCurrentThres = 0.5;
        final double kRpmThres = 300;

        mRightMaster.set(ControlMode.PercentOutput, 0.0);
        mRightSlave.set(ControlMode.PercentOutput,0.0);
        mLeftMaster.set(ControlMode.PercentOutput,0.0);
        mLeftSlave.set(ControlMode.PercentOutput,0.0);

        mRightMaster.set(ControlMode.PercentOutput,-0.25);
        Timer.delay(4.0);
        final double currentRightMaster = mRightMaster.getOutputCurrent();
        final double rpmRightMaster = mRightMaster.getSelectedSensorVelocity();
        mRightMaster.set(ControlMode.PercentOutput,0.0);

        Timer.delay(2.0);

        mRightSlave.set(ControlMode.PercentOutput,-0.25);
        Timer.delay(4.0);
        final double currentRightSlave = mRightSlave.getOutputCurrent();
        final double rpmRightSlave = mRightMaster.getSelectedSensorVelocity();
        mRightSlave.set(ControlMode.PercentOutput,0.0);

        Timer.delay(2.0);

        mLeftMaster.set(ControlMode.PercentOutput,0.25);
        Timer.delay(4.0);
        final double currentLeftMaster = mLeftMaster.getOutputCurrent();
        final double rpmLeftMaster = mLeftMaster.getSelectedSensorVelocity();
        mLeftMaster.set(ControlMode.PercentOutput,0.0f);

        Timer.delay(2.0);

        mLeftSlave.set(ControlMode.PercentOutput,0.25);
        Timer.delay(4.0);
        final double currentLeftSlave = mLeftSlave.getOutputCurrent();
        final double rpmLeftSlave = mLeftMaster.getSelectedSensorVelocity();
        mLeftSlave.set(ControlMode.PercentOutput,0.0);

        
        mRightSlave.set(ControlMode.Follower, Constants.kRightDriveMasterId);

        mLeftSlave.set(ControlMode.Follower, Constants.kLeftDriveMasterId);

        System.out.println("Drive Right Master Current: " + currentRightMaster + " Drive Right Slave Current: "
                + currentRightSlave); 
        System.out.println(
                "Drive Left Master Current: " + currentLeftMaster + " Drive Left Slave Current: " + currentLeftSlave);
        System.out.println("Drive RPM RMaster: " + rpmRightMaster + " RSlave: " + rpmRightSlave + " LMaster: "
                + rpmLeftMaster + " LSlave: " + rpmLeftSlave);

        boolean failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Master Current Low !!!!!!!!!!");
        }

        if (currentRightSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Slave Current Low !!!!!!!!!!");
        }

        if (currentLeftMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Master Current Low !!!!!!!!!!");
        }

        if (currentLeftSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Slave Current Low !!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentRightMaster, currentRightSlave), currentRightMaster,
                5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Currents Different !!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentLeftMaster, currentLeftSlave), currentLeftSlave,
                5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Currents Different !!!!!!!!!!!!!");
        }

        if (rpmRightMaster < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Master RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmRightSlave < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Slave RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmLeftMaster < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Master RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmLeftSlave < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Slave RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(rpmRightMaster, rpmRightSlave, rpmLeftMaster, rpmLeftSlave),
                rpmRightMaster, 250)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!!! Drive RPMs different !!!!!!!!!!!!!!!!!!!");
        }

        return !failure;
    }
}
