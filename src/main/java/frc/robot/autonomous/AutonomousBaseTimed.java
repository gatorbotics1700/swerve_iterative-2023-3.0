package frc.robot.autonomous;

import frc.robot.Robot;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutonomousBaseTimed extends AutonomousBase{
    DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;
    private double timeStart;
    private double timeElapsed = 0;
    final double target = 5.0; //units in seconds

    @Override
    public void init(){
        drivetrainSubsystem.resetOdometry(new Pose2d());
        timeStart = System.currentTimeMillis(); 
    }

    @Override
    public void periodic(){
        timeElapsed = (System.currentTimeMillis() - timeStart)/ 1000;
        if(timeElapsed < target){
            drivetrainSubsystem.setSpeed(new ChassisSpeeds(0.0, 0.3, 0.0));
            drivetrainSubsystem.drive();
        } else {
            drivetrainSubsystem.stopDrive();   
        }
    }
}