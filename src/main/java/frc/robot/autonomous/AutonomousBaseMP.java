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
    private Trajectory.State end;
    private HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0), //TODO: CHANGE KP
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
        timeElapsed = 0;

        double timeCheck = trajectory1.getTotalTimeSeconds();
        end = trajectory1.sample(timeCheck);
        //System.out.println("total time: " + timeCheck);
    }
    
    @Override
    public void periodic(){
        if (doing == Doing.FIRST){
            timeStart = System.currentTimeMillis();
            setDoing(Doing.TRAJECTORY1);
        } else if (doing == Doing.TRAJECTORY1){
            followTrajectory(trajectory1);
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

        timeElapsed = System.currentTimeMillis() - timeStart;
    }

    public boolean trajectoryDone(Trajectory trajectory){

        if(Math.abs(end.poseMeters.getX() - DrivetrainSubsystem.m_pose.getX()) < 0.5 /*/&& 
        Math.abs(end.poseMeters.getY() - DrivetrainSubsystem.m_pose.getY()) < 2 &&
        Math.abs(end.poseMeters.getRotation().getDegrees() - DrivetrainSubsystem.m_pose.getRotation().getDegrees()) < 2 */){
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
        STOP,
        FIRST; 
    }
    public Doing doing = Doing.FIRST; 

    public void setDoing(Doing newDoing){
        doing = newDoing;
    }

    public static Trajectory generateTrajectory(Pose2d starting, Pose2d ending, Translation2d interior1, Translation2d interior2, Translation2d interior3){
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(interior1);
        interiorWaypoints.add(interior2);
        interiorWaypoints.add(interior3);

        SwerveDriveKinematicsConstraint swerveDriveKinematicsConstraint = new SwerveDriveKinematicsConstraint(drivetrainSubsystem.m_kinematics, DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*.75);
        MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*.75);

        TrajectoryConfig config = new TrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*0.75, 1); //4.96, 2.8 //we should maybe look into this further
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
        Trajectory.State goal = trajectory.sample(timeElapsed/1000);
        
        ChassisSpeeds adjustedSpeeds = controller.calculate(
            DrivetrainSubsystem.m_pose, goal, Rotation2d.fromDegrees(0));
        
        drivetrainSubsystem.setSpeed(adjustedSpeeds);
        //System.out.println("Actual time elapsed: " + timeElapsed/1000 + "\n" + "Speed: " + adjustedSpeeds.vxMetersPerSecond + ", " + adjustedSpeeds.vyMetersPerSecond + ", " + adjustedSpeeds.omegaRadiansPerSecond + " Goal endpoint: " + goal);
    }

    

}