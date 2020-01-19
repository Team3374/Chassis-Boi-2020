package frc.robot2020;

// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot2020.auto.AutoModeExecuter;
import frc.robot2020.loops.Looper;
import frc.robot2020.paths.profiles.PathAdapter;
import frc.robot2020.subsystems.*;
import edu.wpi.first.wpilibj.Solenoid;
import frc.util.*;
//import frc.util.math.RigidTransform2d;
import edu.wpi.first.networktables.*;

import java.sql.Timestamp;
import java.util.Arrays;
import java.util.Map;
import java.math.*;

/**
 * The main robot class, which instantiates all robot parts and helper classes and initializes all loops. Some classes
 * are already instantiated upon robot startup; for those classes, the robot gets the instance as opposed to creating a
 * new object
 * 
 * After initializing all robot parts, the code sets up the autonomous and teleoperated cycles and also code that runs
 * periodically inside both routines.
 * 
 * This is the nexus/converging point of the robot code and the best place to start exploring.
 * 
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
    // Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    //private Superstructure mSuperstructure = Superstructure.getInstance();
    //private LED mLED = LED.getInstance();
    //private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;


    // Create subsystem manager
    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(Drive.getInstance()));                    

    // Initialize other helper objects

    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

   
   

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mDrive.zeroSensors();
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
           

            // Pre calculate the paths we use for auto.
            PathAdapter.calculatePaths();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     * 
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());



            zeroAllSensors();
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        allPeriodic();
        overallPeriodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);

            NetworkTable tableBoi = NetworkTableInstance.getDefault().getTable("limelight");
            NetworkTableEntry led = tableBoi.getEntry("ledMode");
            led.setNumber(1);

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void overallPeriodic(){
       
        double turnyBoi = mControlBoard.getTurn()*.4;
        double throttlyBoi = Math.pow(mControlBoard.getRightTrigger()-mControlBoard.getLeftTrigger(),3)*.6;
        double leftPower = throttlyBoi + turnyBoi;
        double rightPower = throttlyBoi - turnyBoi;


        // throttle = 1 power to both
        // turn = less power to one side
        // no throttle and turn left - 1 to left 1 to right
        //no throttle and turn right 1 to left -1 to right
        //wide turn left, full power right, no power left
        mDrive.setOpenLoop(new DriveSignal(leftPower, rightPower));
       
    }
    

    /**
     * This function is called periodically during operator control.
     * 
     * The code uses state machines to ensure that no matter what buttons the driver presses, the robot behaves in a
     * safe and consistent manner.
     * 
     * Based on driver input, the code sets a desired state for each subsystem. Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        try {
            overallPeriodic();
            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            PathAdapter.calculatePaths();

            // If are tuning, dump map so far.

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override

   // @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();

    }
}
