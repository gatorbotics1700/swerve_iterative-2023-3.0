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
    private HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(6.28, Math.PI)));
        // trapezoid profile takes in max rotation velocity and max rotation acceleration 
        // PIDController #1: the first arg rep how many m/s added in the x direction for every meter of error in the x direction
        // PIDController #2 : the first arg rep how many m/s added in the y direction for every meter of error in the y direction
    
    private static DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

    public AutonomousBaseMP(Trajectory trajectory1, Trajectory trajectory2, Trajectory trajectory3){
        this.trajectory1 = trajectory1; 
        this.trajectory2 = trajectory2;
        this.trajectory3 = trajectory3; 
    }

    @Override
    public void init(){
        timeStart = System.currentTimeMillis();
    }
    
    @Override
    public void periodic(){
        timeElapsed = System.currentTimeMillis() - timeStart;
        if (doing == Doing.TRAJECTORY1){
            followTrajectory(trajectory1);
            if (trajectoryDone(trajectory1)){
                setDoing(Doing.TRAJECTORY2);
            }
        } else if (doing == Doing.TRAJECTORY2){
            followTrajectory(trajectory2);
            if (trajectoryDone(trajectory2)){
                setDoing(Doing.TRAJECTORY3);
            }
        } else if (doing == Doing.TRAJECTORY3){
            followTrajectory(trajectory3);
            if (trajectoryDone(trajectory3)){
                setDoing(Doing.STOP);
            }
        } else if (doing == Doing.PLACEHIGH){
            
        } else if (doing == Doing.PICKUP){
            
        } else if (doing == Doing.BALANCE){
            
        } else {
            drivetrainSubsystem.stopDrive();
        }
    }

    public boolean trajectoryDone(Trajectory trajectory){
        double timeCheck = trajectory.getTotalTimeSeconds();
        Trajectory.State end = trajectory.sample(timeCheck);
        if(Math.abs(end.poseMeters.getX() - DrivetrainSubsystem.m_pose.getX()) < 2 && 
            Math.abs(end.poseMeters.getY() - DrivetrainSubsystem.m_pose.getY()) < 2 &&
            Math.abs(end.poseMeters.getRotation().getDegrees() - DrivetrainSubsystem.m_pose.getRotation().getDegrees()) < 2){
            return true;
        }
        return false;
    }

    public static enum Doing{
        TRAJECTORY1,
        TRAJECTORY2, 
        TRAJECTORY3,
        PLACEHIGH, 
        PICKUP, 
        BALANCE,
        STOP;  
    }
    public Doing doing = Doing.TRAJECTORY1; 

    public void setDoing(Doing newDoing){
        doing = newDoing;
    }

    public static Trajectory generateTrajectory(Pose2d starting, Pose2d ending, Translation2d interior1, Translation2d interior2, Translation2d interior3){
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(interior1);
        interiorWaypoints.add(interior2);
        interiorWaypoints.add(interior3);

        SwerveDriveKinematicsConstraint swerveDriveKinematicsConstraint = new SwerveDriveKinematicsConstraint(drivetrainSubsystem.m_kinematics, DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
        MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);

        TrajectoryConfig config = new TrajectoryConfig(4.96, 2.8); //we should maybe look into this further
        config.addConstraint(swerveDriveKinematicsConstraint);
        config.addConstraint(maxVelocityConstraint);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            starting,
            interiorWaypoints, 
            ending,
            config
        );
        return trajectory;
    }

    public void followTrajectory(Trajectory trajectory){
        Trajectory.State goal = trajectory.sample(timeElapsed);

        ChassisSpeeds adjustedSpeeds = controller.calculate(
            DrivetrainSubsystem.m_pose, goal, Rotation2d.fromDegrees(0));
        
        drivetrainSubsystem.setSpeed(adjustedSpeeds);
    }

    

}