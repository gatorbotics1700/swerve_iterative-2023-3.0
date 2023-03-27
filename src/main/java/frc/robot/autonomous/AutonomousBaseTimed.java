package frc.robot.autonomous;

import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutonomousBaseTimed extends AutonomousBase{
    private DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;
    private double timeStart;
    private double timeElapsed = 0;
    final double target = 10.0; //units in seconds


    @Override
    public void init(){
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
