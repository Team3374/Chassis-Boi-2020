/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class PID_Calculator {
    private double P;
    private double I;
    private double D;
    private double PreviousError;
    private boolean RunningState;
    private double PreviousTime;
    private double PreviousOutput;
    public PID_Calculator(double Pin, double Iin, double Din){
        this.P = Pin;
        this.I = Iin;
        this.D = Din;
        this.PreviousError = 0;
        this.RunningState = false;
    }
    public void Reset(){
        RunningState = false;
    }
    public double PIDCalc(double CurrentVal, double WantedVal, double timestamp){
        double PError = 0;
        double IError = 0;
        double DError = 0;
        double CurrentError = CurrentVal - WantedVal;
        double DMultiplier = 1; //.425 .211693409
        if(RunningState){
            //if running keep running
            if(CurrentError == PreviousError){
                return PreviousOutput;
            }
            PError = CurrentError * P;
            DError = ((CurrentError - PreviousError)/(timestamp - PreviousTime)) * D;
            PreviousError = DMultiplier * CurrentError + (1 - DMultiplier) * PreviousError;
            PreviousTime = timestamp;
        }
        else{
            RunningState = true;
            PError = CurrentError * P;
            PreviousError = CurrentError;
            PreviousTime = timestamp;
        }
        double[] pidArray = {PError, IError, DError};
        SmartDashboard.putNumberArray("PID", pidArray);
        //System.out.println(Double.toString(PError) + "," + Double.toString(DError));
        PreviousOutput = PError + IError + DError;
        return PreviousOutput;
    }
}
