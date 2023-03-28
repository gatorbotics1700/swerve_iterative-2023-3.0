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

    private boolean firstTime;
    private double startingTime;
    private double desireTime;
    private double desiredAngle;
    private DrivetrainSubsystem drivetrainSubsystem;
    private Mechanisms mechanisms; 

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

    public AutonomousBaseEngage (int option){
        init();
        if(option==0){
            setState(AutoEngageStates.DRIVE);
        } else if (option==1){
            setState(AutoEngageStates.LOW_NODE);
        } else {
            setState(AutoEngageStates.MID_NODE);
        }
    }

    @Override
    public void init(){
       drivetrainSubsystem = Robot.m_drivetrainSubsystem;
       startingTime = 0;
       mechanisms = Mechanisms.m_mechanisms;
       firstTime = true;
       desireTime = 5000;
       desiredAngle = 10;
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
                System.out.println(startingTime);
            }
            if(/*startingTime + desireTime >= System.currentTimeMillis() ||*/ Math.abs(drivetrainSubsystem.getPitch()) > desiredAngle){
                setState(AutoEngageStates.ENGAGE);
            } else{
                drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, drivetrainSubsystem.getPoseRotation()));
            }
        } else if(autoEngageState == AutoEngageStates.ENGAGE){
            drivetrainSubsystem.pitchBalance(0.0);
        }else{
            drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrainSubsystem.getPoseRotation()));
        }
    }
}