package frc.robot.autonomous;

import java.util.ArrayList;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import frc.robot.autonomous.MPStateWithCoordinate.MPStates;
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
    private Trajectory trajectory1;
    private Trajectory trajectory2;
    private Trajectory trajectory3;
    private Trajectory trajectory4;
    private Trajectory.State end;
    private HolonomicDriveController controller;
        // trapezoid profile takes in max rotation velocity and max rotation acceleration 
        // PIDController #1: the first arg rep how many m/s added in the x direction for every meter of error in the x direction
        // PIDController #2 : the first arg rep how many m/s added in the y direction for every meter of error in the y direction
    
    private static DrivetrainSubsystem drivetrainSubsystem;
    public MPStates mpStates;
    private Mechanisms mechanisms;
    private AutonomousBaseEngage autoEngage; 
    private MPStateWithCoordinate[] MPStateSequence;
    private Pose2d mpStartingCoordinate;

    private Boolean isFirst;
    private double startTime;
    private int i;

    public AutonomousBaseMP(Pose2d mpStartingCoordinate, MPStateWithCoordinate[] MPStateSequence){
        this.mpStartingCoordinate = mpStartingCoordinate;
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
        double timeCheck = trajectory1.getTotalTimeSeconds();
        end = trajectory1.sample(timeCheck);
        //Avery note: might need to edit this when we work with multiple trajectories bc we only check Traj one 
        //maybe a method to reset time that can be called when we switch to a new trajectory in the else ifs 
        System.out.println("total time: " + timeCheck);
        System.out.println("total time: " + timeCheck);
        System.out.println("Init pose: " + drivetrainSubsystem.getMPoseX());
        drivetrainSubsystem.resetOdometry(new Pose2d());
        //System.out.println("Traj 1 " + trajectory1 +  "/n Traj 2 " + trajectory2 + "/n Traj 3 " + trajectory3); 
    }

    private MPStates mpstates = MPStates.FIRST; 
    
    @Override
    //Avery note: make state machine work better!! Like PID 
    public void periodic(){
        mpstates = MPStateSequence[i].mpState;
        System.out.println("state: " + mpstates);
        if(mpstates == MPStates.FIRST){
            timeStart = System.currentTimeMillis();
            setStates(MPStates.TRAJECTORY1); //Anaika notes: fix this => they should all be one trajectory bc all working mp paths only need 1
            System.out.println("Doing first");
            System.out.println("initial pose: " + drivetrainSubsystem.getMPoseX());
            i++;
            System.out.println("moving on to " + MPStateSequence[i]);
        } else if(mpstates == MPStates.TRAJECTORY1){
            System.out.println("traj 1 End X: "+ end.poseMeters.getX() + " trajectory 1 Get X: " + drivetrainSubsystem.getMPoseX()); 
            followTrajectory(trajectory1);
            System.out.println("Doing Trajectory 1"); 
            if(trajectoryDone(trajectory1)){
                System.out.println("STOP");
                setStates(MPStates.STOP);
                i++;
                System.out.println("moving on to " + MPStateSequence[i]);
            }
        } else if(mpstates == MPStates.TRAJECTORY2){
            followTrajectory(trajectory2);
            if(trajectoryDone(trajectory2)){
                setStates(MPStates.TRAJECTORY3);
                i++;
                System.out.println("moving on to " + MPStateSequence[i]);
            }
            
        } else if(mpstates == MPStates.TRAJECTORY3){
            System.out.println("traj 3 End X: "+ end.poseMeters.getX() + " trajectory 3 Get X: " + drivetrainSubsystem.getMPoseX()); 
            followTrajectory(trajectory3);
            if(trajectoryDone(trajectory3)){
                setStates(MPStates.TRAJECTORY4);
                i++;
                System.out.println("moving on to " + MPStateSequence[i]);
            }
            
        } else if(mpstates == MPStates.TRAJECTORY4){
            followTrajectory(trajectory4);
            if(trajectoryDone(trajectory4)){
                setStates(MPStates.STOP);
                i++;
                System.out.println("moving on to " + MPStateSequence[i]);
            }
            
        } else if(mpstates == MPStates.MID){
            System.out.println("mid node");
            setStates(MPStates.MID);
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
           
        } else if(mpstates == MPStates.LOW){
            System.out.println("low node");
            setStates(MPStates.LOW);
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
            
        } else if(mpstates == MPStates.BALANCE){
            autoEngage.periodic();   
        } else{
            drivetrainSubsystem.stopDrive();
        }

        timeElapsed = System.currentTimeMillis() - timeStart;
    }

    public boolean trajectoryDone(Trajectory trajectory){
        double error = Math.abs(end.poseMeters.getX() - drivetrainSubsystem.getMPoseX());
        System.out.println("error: " + error);
        System.out.println("End X: "+ end.poseMeters.getX() + " Get X: " + drivetrainSubsystem.getMPoseX()); 
        if(error < 0.1 /*/&& 
        Math.abs(end.poseMeters.getY() - DrivetrainSubsystem.m_pose.getY()) < 2 &&
        Math.abs(end.poseMeters.getRotation().getDegrees() - DrivetrainSubsystem.m_pose.getRotation().getDegrees()) < 2 */){
            return true;
        }else{
            return false;
        }
    }

    public void setStates(MPStates newState){
        mpstates = newState;
    }

    public void followTrajectory(Trajectory trajectory){
        Trajectory.State goal = trajectory.sample(timeElapsed/1000);
        System.out.println("follow End X: "+ end.poseMeters.getX() + " follow Get X: " + drivetrainSubsystem.getMPoseX()); 
        
        ChassisSpeeds adjustedSpeeds = controller.calculate(
            drivetrainSubsystem.getMPose(), goal, end.poseMeters.getRotation());
            //change angle to get the trajectory angle later
        
        drivetrainSubsystem.setSpeed(adjustedSpeeds);
    }

}
