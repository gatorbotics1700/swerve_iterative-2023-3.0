package frc.robot.autonomous;

import java.util.ArrayList;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import frc.robot.autonomous.MPState.MPStateLabel;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import frc.robot.subsystems.Mechanisms;


public class AutonomousBaseMP extends AutonomousBase{
    private double timeStart;
    private double timeElapsed = 0;
    private Trajectory.State currentTrajState;
    private HolonomicDriveController controller;
        // trapezoid profile takes in max rotation velocity and max rotation acceleration 
        // PIDController #1: the first arg rep how many m/s added in the x direction for every meter of error in the x direction
        // PIDController #2 : the first arg rep how many m/s added in the y direction for every meter of error in the y direction
    
    private static DrivetrainSubsystem drivetrainSubsystem;
    public MPStateLabel mpStates;
    private Mechanisms mechanisms;
    private AutonomousBaseEngage autoEngage; 
    private MPState[] MPStateSequence;

    private Boolean isFirst;
    private double startTime;
    private int i;

    public AutonomousBaseMP(MPState[] MPStateSequence){
        System.out.println("In autobase structure!");
        this.MPStateSequence =  MPStateSequence;
        controller = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0), //TODO: CHANGE KP
            new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(6.28, Math.PI)));
        drivetrainSubsystem = Robot.m_drivetrainSubsystem;
        init();
    }

    @Override
    public void init(){
        timeStart = 0.0;
        i = 0;
        isFirst = true;
        System.out.println("Init pose: " + drivetrainSubsystem.getMPoseX());
        drivetrainSubsystem.resetOdometry(new Pose2d());
        //System.out.println("Traj 1 " + trajectory1 +  "/n Traj 2 " + trajectory2 + "/n Traj 3 " + trajectory3); 
    }

    private MPStateLabel currentStateLabel = MPStateLabel.FIRST; 
    
    @Override
    public void periodic(){
        currentStateLabel = MPStateSequence[i].stateLabel;
        System.out.println("state: " + currentStateLabel);
        if(currentStateLabel == MPStateLabel.FIRST){
            timeStart = System.currentTimeMillis();
            setStates(MPStateLabel.TRAJECTORY);  // made one state for trajectory and are putting individual trajectories in followTradjectory()
            System.out.println("Doing first");
            System.out.println("initial pose: " + drivetrainSubsystem.getMPoseX());
            i++;
            System.out.println("moving on to " + MPStateSequence[i]);
        } else if(currentStateLabel == MPStateLabel.TRAJECTORY){
            if(MPStateSequence[i].trajectory == null){
                System.out.println("No trajectory");
                setStates(MPStateLabel.STOP);
                i++;
                System.out.println("moving on to " + MPStateSequence[i]);
            }else{
                double timeCheck;
                timeCheck = MPStateSequence[i].trajectory.getTotalTimeSeconds();
                currentTrajState = MPStateSequence[i].trajectory.sample(timeCheck);
                System.out.println("total time: " + timeCheck);
                System.out.println("trajectory End X: "+ currentTrajState.poseMeters.getX() + " trajectory Get X: " + drivetrainSubsystem.getMPoseX()); 
                followTrajectory(MPStateSequence[i].trajectory); 
                if(trajectoryDone(MPStateSequence[i].trajectory)){
                    System.out.println("STOP");
                    setStates(MPStateLabel.STOP);
                    i++;
                    System.out.println("moving on to " + MPStateSequence[i]);
                }
            }
        } else if(currentStateLabel == MPStateLabel.MID){
            System.out.println("mid node");
            setStates(MPStateLabel.MID);
            if(mechanisms.isDoneMid()==true){
                if(isFirst){
                    startTime = System.currentTimeMillis();
                    isFirst = false;
                }
                mechanisms.pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeStates.RELEASING);
                if(System.currentTimeMillis()-startTime>=1000){ //time to outtake before moving on
                    i++;
                    isFirst = true;
                }
            } 
           
        } else if(currentStateLabel == MPStateLabel.LOW){
            System.out.println("low node");
            setStates(MPStateLabel.LOW);
            if(isFirst){
                timeStart = System.currentTimeMillis();
                isFirst = false;
            }
            if(mechanisms.isDoneLow()==true){
                mechanisms.pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeStates.RELEASING);
                if(System.currentTimeMillis()-startTime>=1000){
                    i++;
                    isFirst = true;
                }
            }
            
        } else if(currentStateLabel == MPStateLabel.ENGAGE){
            autoEngage.periodic();   
        } else{
            drivetrainSubsystem.stopDrive();
           // System.out.println("DID NOT RECOGNIZE STATE LABEL; STOP DRIVE!!!");
        }

        timeElapsed = System.currentTimeMillis() - timeStart;
    }

    public boolean trajectoryDone(Trajectory trajectory){
        double error = Math.abs(currentTrajState.poseMeters.getX() - drivetrainSubsystem.getMPoseX());
        System.out.println("error: " + error);
        System.out.println("End X: "+ currentTrajState.poseMeters.getX() + " Get X: " + drivetrainSubsystem.getMPoseX()); 
        if(error < 0.1 /*/&& 
        Math.abs(end.poseMeters.getY() - DrivetrainSubsystem.m_pose.getY()) < 2 &&
        Math.abs(end.poseMeters.getRotation().getDegrees() - DrivetrainSubsystem.m_pose.getRotation().getDegrees()) < 2 */){
            return true;
        }else{
            return false;
        }
    }

    public void setStates(MPStateLabel newStateLabel){
        currentStateLabel = newStateLabel;
    }

    public void followTrajectory(Trajectory trajectory){
        Trajectory.State goal = trajectory.sample(timeElapsed/1000);
        System.out.println("follow End X: "+ currentTrajState.poseMeters.getX() + " follow Get X: " + drivetrainSubsystem.getMPoseX()); 
        
        ChassisSpeeds adjustedSpeeds = controller.calculate(
            drivetrainSubsystem.getMPose(), goal, currentTrajState.poseMeters.getRotation());
            //change angle to get the trajectory angle later
        
        drivetrainSubsystem.setSpeed(adjustedSpeeds);
        System.out.println("Adjusted speed: " + adjustedSpeeds);
    }

}
