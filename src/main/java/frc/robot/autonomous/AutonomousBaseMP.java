package frc.robot.autonomous;

import java.util.ArrayList;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;

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

    public AutonomousBaseMP(Trajectory trajectory1, Trajectory trajectory2, Trajectory trajectory3, Trajectory trajectory4){
        this.trajectory1 = trajectory1; 
        this.trajectory2 = trajectory2;
        this.trajectory3 = trajectory3;
        this.trajectory4 = trajectory4; 
        controller = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0), //TODO: CHANGE KP
            new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(6.28, Math.PI)));
        drivetrainSubsystem = Robot.m_drivetrainSubsystem;
        init();
    }

    @Override
    public void init(){
        timeStart = System.currentTimeMillis();
        timeElapsed = 0;

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

    public static enum Doing{
        TRAJECTORY1,
        TRAJECTORY2, 
        TRAJECTORY3,
        TRAJECTORY4,
        PLACEHIGH, 
        PICKUP, 
        BALANCE,
        STOP,
        FIRST; 
    }

    private Doing doing = Doing.FIRST; 
    
    @Override
    //Avery note: make state machine work better!! Like PID 
    public void periodic(){
        if (doing == Doing.FIRST){
            timeStart = System.currentTimeMillis();
            setDoing(Doing.TRAJECTORY1);
            System.out.println("Doing first");
            System.out.println("initial pose: " + drivetrainSubsystem.getMPoseX());
        } else if (doing == Doing.TRAJECTORY1){
            System.out.println("traj 1 End X: "+ end.poseMeters.getX() + " traj 1 Get X: " + drivetrainSubsystem.getMPoseX()); 
            followTrajectory(trajectory1);
            System.out.println("Doing Traj 1"); 
            if (trajectoryDone(trajectory1)){
                 System.out.println("STOP");
                 setDoing(Doing.STOP);
            }
        } else if (doing == Doing.TRAJECTORY2){
            followTrajectory(trajectory2);
            if (trajectoryDone(trajectory2)){
                setDoing(Doing.TRAJECTORY3);
            }
        } else if (doing == Doing.TRAJECTORY3){
            System.out.println("traj 3 End X: "+ end.poseMeters.getX() + " traj 3 Get X: " + drivetrainSubsystem.getMPoseX()); 
            followTrajectory(trajectory3);
            if (trajectoryDone(trajectory3)){
                setDoing(Doing.TRAJECTORY4);
            }
        } else if (doing == Doing.TRAJECTORY4){
            followTrajectory(trajectory4);
            if (trajectoryDone(trajectory4)){
                setDoing(Doing.STOP);
            }
        } else if (doing == Doing.PLACEHIGH){
            
        } else if (doing == Doing.PICKUP){
            
        } else if (doing == Doing.BALANCE){
            
        } else {
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

    public void setDoing(Doing newDoing){
        doing = newDoing;
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
