package frc.robot.autonomous;

import javax.swing.SwingWorker.StateValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants; 
import frc.robot.autonomous.StateWithCoordinate;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.Vision.LimeLightSubsystem;
import frc.robot.subsystems.Vision.AprilTagSubsystem;

public class AutonomousBaseEngage extends AutonomousBase{

    public boolean firstTime;
    public double startingTime;
    public double desireTime;
    public double desiredAngle;
    DrivetrainSubsystem drivetrainSubsystem;
    Mechanisms mechanisms; 

    public enum AutoEngageStates{
        ENGAGE,
        DRIVE,
        MID_NODE,
        LOW_NODE,
        OFF;
    }

    public static AutoEngageStates autoEngageState = AutoEngageStates.OFF;

    public void setState(AutoEngageStates newEngageState){
        autoEngageState = newEngageState;
    }

    @Override
    public void init(){
       drivetrainSubsystem = Robot.m_drivetrainSubsystem;
       startingTime = 0;
       mechanisms = Mechanisms.m_mechanisms;
       firstTime = true;

    }

    @Override
    public void periodic(){
        if(autoEngageState == AutoEngageStates.MID_NODE){
            mechanisms.setState(MechanismStates.MID_NODE);
            if(mechanisms.isDoneMid()){
                firstTime = true; 
                setState(AutoEngageStates.DRIVE);
            }
        } else if(autoEngageState == AutoEngageStates.LOW_NODE){
            mechanisms.setState(MechanismStates.LOW_NODE);
            if(mechanisms.isDoneLow()){
                firstTime = true; 
                setState(AutoEngageStates.DRIVE);
            }
        } else if(autoEngageState == AutoEngageStates.DRIVE){
            if (firstTime == true){
                firstTime = false;
                startingTime = System.currentTimeMillis();
            }
            if(startingTime >= desireTime || drivetrainSubsystem.getPitch() > desiredAngle){
                setState(AutoEngageStates.ENGAGE);
            } else{
                drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, drivetrainSubsystem.getPoseRotation()));
            }
        } else if(autoEngageState == AutoEngageStates.ENGAGE){
            drivetrainSubsystem.pitchBalance(0.0);
        }else{
            drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrainSubsystem.getPoseRotation()));
        }
        drivetrainSubsystem.drive();
    }
}