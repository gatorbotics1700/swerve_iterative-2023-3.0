package frc.robot.autonomous;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

//IN METERS!
public class Trajectories{
    private static double mpi = Constants.METERS_PER_INCH; 

    public static Trajectory uno = generateTrajectory(
        new Pose2d(), 
        new Pose2d(2.0, 0.0, new Rotation2d(Math.toRadians(90))), 
        new Translation2d(1.0, 0.0), 
        new Translation2d(1.2, 0.25), 
        new Translation2d(1.4, 0.0),
        false
    );

    public static Trajectory dos = generateTrajectory(
        new Pose2d(), 
        new Pose2d(1.5, 2.3, new Rotation2d(0)), 
        new Translation2d(.2, .5), 
        new Translation2d(.9, 1.5), 
        new Translation2d(1.5, 2.0), 
        false
    );

    public static Trajectory tres = generateTrajectory(
        new Pose2d(), 
        new Pose2d(2.0, -1.0, new Rotation2d(0)), 
        new Translation2d(.2, -.4), 
        new Translation2d(.9, -.6), 
        new Translation2d(1.5, -.8),
        false
    );

    public static Trajectory oneHD3R = generateTrajectory(
        new Pose2d(595.614, 20.19, new Rotation2d(Math.toRadians(180))), 
        new Pose2d(372.684, 37.193, new Rotation2d(Math.toRadians(0))),
        new Translation2d(520.0, 25.2), 
        new Translation2d(445.0, 30.2), 
        new Translation2d(400.0, 33.19), 
        false
    ); 

    public static Trajectory twoHD3R = generateTrajectory(
        new Pose2d(372.684, 37.193, new Rotation2d(Math.toRadians(0))), 
        new Pose2d(595.461, 43.068, new Rotation2d(Math.toRadians(180))),
        new Translation2d(400.0, 33.19), 
        new Translation2d(445.0, 30.2), 
        new Translation2d(520.0, 25.2), 
        false 
    ); 

    public static Trajectory threeHD3R = generateTrajectory(
        new Pose2d(595.461, 43.068, new Rotation2d(Math.toRadians(180))),
        new Pose2d(372.606,85.622, new Rotation2d(Math.toRadians(0))),
        new Translation2d(525, 43.5),
        new Translation2d(454.199, 45.934), 
        new Translation2d(400.0, 55.0),
        false
    ); 

    public static Trajectory fourHD3R = generateTrajectory(
        new Pose2d(372.606,85.622, new Rotation2d(Math.toRadians(0))),
        new Pose2d(595.529, 66.117, new Rotation2d(Math.toRadians(180))),
        new Translation2d(400.0, 55.0),
        new Translation2d(454.199, 45.934),
        new Translation2d(525, 43.5),
        false
    ); 

    public static Trajectory flowerOne = generateTrajectory(
        new Pose2d(0, 20 * mpi, new Rotation2d(Math.toRadians(0))),
        new Pose2d(11 * mpi, 2 * mpi, new Rotation2d(Math.toRadians(0))),
        new Translation2d(2.69 * mpi, 16.24 * mpi),
        new Translation2d(4.58 * mpi, 10.44 * mpi),
        new Translation2d(6.22 * mpi, 5.18 * mpi),
        false
    ); 

    public static Trajectory flowerTwo = generateTrajectory(
        new Pose2d(11 * mpi, 2 * mpi, new Rotation2d(Math.toRadians(0))),
        new Pose2d(27.4 * mpi, 15.4 * mpi, new Rotation2d(Math.toRadians(0))),
        new Translation2d(16.52 * mpi, 1.65 * mpi),
        new Translation2d(23.08 * mpi, 5.07 * mpi),
        new Translation2d(25.3 * mpi, 10.42 * mpi),
        false 
    ); 

    public static Trajectory flowerThree = generateTrajectory(
        new Pose2d(11 * mpi, 2 * mpi, new Rotation2d(Math.toRadians(0))),
        new Pose2d(27.4 * mpi, 15.4 * mpi, new Rotation2d(Math.toRadians(0))),
        new Translation2d(16.52 * mpi, 1.65 * mpi),
        new Translation2d(23.08 * mpi, 5.07 * mpi),
        new Translation2d(25.3 * mpi, 10.42 * mpi),
        true
    ); 

    public static Trajectory flowerFour = generateTrajectory(
        new Pose2d(0, 20 * mpi, new Rotation2d(Math.toRadians(0))),
        new Pose2d(11 * mpi, 2 * mpi, new Rotation2d(Math.toRadians(0))),
        new Translation2d(2.69 * mpi, 16.24 * mpi),
        new Translation2d(4.58 * mpi, 10.44 * mpi),
        new Translation2d(6.22 * mpi, 5.18 * mpi),
        true
    ); 

    public static Trajectory nada = generateTrajectory(
        new Pose2d(),
        new Pose2d(),
        new Translation2d(),
        new Translation2d(),
        new Translation2d(),
        false
    ); 

    public static Trajectory generateTrajectory(Pose2d starting, Pose2d ending, Translation2d interior1, Translation2d interior2, Translation2d interior3, boolean isReversed){
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(interior1);
        interiorWaypoints.add(interior2);
        interiorWaypoints.add(interior3);

        SwerveDriveKinematicsConstraint swerveDriveKinematicsConstraint = new SwerveDriveKinematicsConstraint(DrivetrainSubsystem.getMKinematics(), DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*.75);
        MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*.75);

        TrajectoryConfig config = new TrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*0.75, 1); //4.96, 2.8 //we should maybe look into this further
        System.out.println("Start config pose: " + Robot.m_drivetrainSubsystem.getMPoseX());
        config.addConstraint(swerveDriveKinematicsConstraint);
        config.addConstraint(maxVelocityConstraint);
        config.setReversed(isReversed); 

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            starting,
            interiorWaypoints, 
            ending,
            config
        );
        System.out.println("End config pose: " + Robot.m_drivetrainSubsystem.getMPoseX());
        return trajectory;
    }
}