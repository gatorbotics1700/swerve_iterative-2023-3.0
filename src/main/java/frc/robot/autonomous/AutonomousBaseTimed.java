package frc.robot.autonomous;

import frc.robot.Robot;
import frc.robot.subsystems.*;

public class AutonomousBaseTimed extends AutonomousBase{
    DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;
    private long timeStart;
    private long timeElapsed;
    private double target; //units in seconds

    @Override
    public void init(){
        drivetrainSubsystem.resetOdometry();

        target = 5.0;
        timeStart = System.currentTimeMillis();
        timeElapsed = (System.currentTimeMillis() - timeStart)/ 1000; 
    }

    @Override
    public void periodic(){
        timeElapsed = (System.currentTimeMillis() - timeStart)/ 1000;
        if(timeElapsed < target){
            drivetrainSubsystem.drive();
        } else {
            drivetrainSubsystem.stopDrive();   
        }
    }
}
