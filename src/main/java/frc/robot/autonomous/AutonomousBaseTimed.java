package frc.robot.autonomous;

import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutonomousBaseTimed extends AutonomousBase{
    private DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;
    private double timeStart;
    private double target; //units in seconds

    public AutonomousBaseTimed(){
        init();
    }

    @Override
    public void init(){
        target = 10.0; //units in seconds
        timeStart = System.currentTimeMillis(); 
    }

    @Override
    public void periodic(){
        double timeElapsed = (System.currentTimeMillis() - timeStart)/ 1000;
        if(timeElapsed < target){
            drivetrainSubsystem.setSpeed(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0.2, 0.0, 0.0, drivetrainSubsystem.getPoseRotation()
                )
            );
            drivetrainSubsystem.drive();
        } else {
            drivetrainSubsystem.stopDrive();   
        }
    }
}
