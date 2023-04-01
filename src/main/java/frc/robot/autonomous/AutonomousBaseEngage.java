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
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem;

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
        DRIVEBACK,
        MID_NODE,
        LOW_NODE,
        OFF;
    }

    private AutoEngageStates autoEngageState;

    public void setState(AutoEngageStates newEngageState){
        autoEngageState = newEngageState;
    }

    public AutonomousBaseEngage (int option){
        init();
        if(option==0){
            setState(AutoEngageStates.DRIVE);
            System.out.println("OPTION IS ZERO");
        } else if (option==1){
            setState(AutoEngageStates.LOW_NODE);
        } else if (option==2) {
            setState(AutoEngageStates.MID_NODE);
        } else {
            setState(AutoEngageStates.DRIVEBACK);
        }
    }

    @Override
    public void init(){
        System.out.println("IN AUTO ENGAGE INIT");
       drivetrainSubsystem = Robot.m_drivetrainSubsystem;
       startingTime = 0;
       mechanisms = Robot.m_mechanisms;
       firstTime = true;
       desireTime = 5000;
       desiredAngle = 15;
       autoEngageState = AutoEngageStates.OFF;
    }

    @Override
    public void periodic(){
        System.out.println("Engage state: " + autoEngageState);
        if(autoEngageState == AutoEngageStates.MID_NODE){
            mechanisms.setState(MechanismStates.MID_NODE);
            if(mechanisms.isDoneMid()){
                if(firstTime == true){
                    firstTime = false;
                    startingTime = System.currentTimeMillis();
                }
                mechanisms.pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeStates.RELEASING);
                if (System.currentTimeMillis()-startingTime>=1000){ //time to outtake before moving on
                    setState(AutoEngageStates.DRIVE);
                    mechanisms.setState(MechanismStates.HOLDING);
                }
                
            }
        } else if(autoEngageState == AutoEngageStates.LOW_NODE){
            mechanisms.setState(MechanismStates.LOW_NODE);
            if(mechanisms.isDoneLow()){
                if(firstTime == true){
                    firstTime = false;
                    startingTime = System.currentTimeMillis();
                }
                mechanisms.pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeStates.RELEASING);
                if (System.currentTimeMillis()-startingTime>=1000){ //time to outtake before moving on
                    setState(AutoEngageStates.DRIVE);
                    mechanisms.setState(MechanismStates.HOLDING);
                }
            }
        } else if(autoEngageState == AutoEngageStates.DRIVE){
            //System.out.println("IN DRIVING FOR AUTO TIMED");
            if (firstTime == true){
                firstTime = false;
                startingTime = System.currentTimeMillis();
                System.out.println(startingTime);
            }
            if(/*startingTime + desireTime >= System.currentTimeMillis() ||*/ Math.abs(drivetrainSubsystem.getPitch()) > desiredAngle){
                setState(AutoEngageStates.ENGAGE);
                firstTime = true;
            } else{
                drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, drivetrainSubsystem.getPoseRotation()));
            }
        } else if(autoEngageState == AutoEngageStates.DRIVEBACK){
            //System.out.println("IN DRIVING FOR AUTO TIMED");
            if (firstTime == true){
                firstTime = false;
                startingTime = System.currentTimeMillis();
                System.out.println(startingTime);
            }
            if(/*startingTime + desireTime >= System.currentTimeMillis() ||*/ Math.abs(drivetrainSubsystem.getPitch()) > desiredAngle){
                setState(AutoEngageStates.ENGAGE);
            } else{
                drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(-0.5, 0, 0, drivetrainSubsystem.getPoseRotation()));
            }
        } else if(autoEngageState == AutoEngageStates.ENGAGE){
            drivetrainSubsystem.pitchBalance(0.0);
            mechanisms.setState(MechanismStates.HOLDING);
        }else{
            drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrainSubsystem.getPoseRotation()));
        }
    }
}