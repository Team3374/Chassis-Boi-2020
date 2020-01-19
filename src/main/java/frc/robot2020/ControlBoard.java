package frc.robot2020;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Contains the button mappings for the competition control board. Like the drive code, one instance of the ControlBoard
 * object is created upon startup, then other methods request the singleton ControlBoard instance. Implements the
 * ControlBoardInterface.
 * 
 * @see ControlBoardInterface.java
 */
public class ControlBoard implements ControlBoardInterface {
    private static ControlBoardInterface mInstance = null;

    private static final boolean kUseGamepad = false;

    private XboxController mDriver;

    public static ControlBoardInterface getInstance() {
        if (mInstance == null){
         mInstance = new ControlBoard();
        }
        
        return mInstance;
    }


    protected ControlBoard() {
        
        mDriver = new XboxController(0);
    }

    // DRIVER CONTROLS
    @Override
    public double getRightTrigger() {
      return mDriver.getTriggerAxis(Hand.kRight);
    }

    public double getLeftTrigger() {
      return mDriver.getTriggerAxis(Hand.kLeft);
    }



    @Override
    public double getTurn() {
      return mDriver.getX(Hand.kLeft);
    }

    


    
}
