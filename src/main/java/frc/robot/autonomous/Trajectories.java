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
        new Translation2d(1.4, 0.0)
    );

    public static Trajectory dos = generateTrajectory(
        new Pose2d(), 
        new Pose2d(1.5, 2.3, new Rotation2d(0)), 
        new Translation2d(.2, .5), 
        new Translation2d(.9, 1.5), 
        new Translation2d(1.5, 2.0)
    );

    public static Trajectory tres = generateTrajectory(
        new Pose2d(), 
        new Pose2d(2.0, -1.0, new Rotation2d(0)), 
        new Translation2d(.2, -.4), 
        new Translation2d(.9, -.6), 
        new Translation2d(1.5, -.8)
    );

    public static Trajectory oneHD3R = generateTrajectory(
        new Pose2d(595.614, 20.19, new Rotation2d(Math.toRadians(180))), 
        new Pose2d(372.684, 37.193, new Rotation2d(Math.toRadians(0))),
        new Translation2d(520.0, 25.2), 
        new Translation2d(445.0, 30.2), 
        new Translation2d(400.0, 33.19)
    ); 

    public static Trajectory twoHD3R = generateTrajectory(
        new Pose2d(372.684, 37.193, new Rotation2d(Math.toRadians(0))), 
        new Pose2d(595.461, 43.068, new Rotation2d(Math.toRadians(180))),
        new Translation2d(400.0, 33.19), 
        new Translation2d(445.0, 30.2), 
        new Translation2d(520.0, 25.2) 
    ); 

    public static Trajectory threeHD3R = generateTrajectory(
        new Pose2d(595.461, 43.068, new Rotation2d(Math.toRadians(180))),
        new Pose2d(372.606,85.622, new Rotation2d(Math.toRadians(0))),
        new Translation2d(525, 43.5),
        new Translation2d(454.199, 45.934), 
        new Translation2d(400.0, 55.0)
    ); 

    public static Trajectory fourHD3R = generateTrajectory(
        new Pose2d(372.606,85.622, new Rotation2d(Math.toRadians(0))),
        new Pose2d(595.529, 66.117, new Rotation2d(Math.toRadians(180))),
        new Translation2d(400.0, 55.0),
        new Translation2d(454.199, 45.934),
        new Translation2d(525, 43.5)
    ); 

    public static Trajectory flowerOne = generateTrajectory(
        new Pose2d(0, 20 * mpi, new Rotation2d(Math.toRadians(0))),
        new Pose2d(11 * mpi, 2 * mpi, new Rotation2d(Math.toRadians(0))),
        new Translation2d(2.69 * mpi, 16.24 * mpi),
        new Translation2d(4.58 * mpi, 10.44 * mpi),
        new Translation2d(6.22 * mpi, 5.18 * mpi)
    ); 

    public static Trajectory flowerTwo = generateTrajectory(
        new Pose2d(11 * mpi, 2 * mpi, new Rotation2d(Math.toRadians(0))),
        new Pose2d(27.4 * mpi, 15.4 * mpi, new Rotation2d(Math.toRadians(0))),
        new Translation2d(16.52 * mpi, 1.65 * mpi),
        new Translation2d(23.08 * mpi, 5.07 * mpi),
        new Translation2d(25.3 * mpi, 10.42 * mpi) 
    ); 

    public static Trajectory flowerThree = generateTrajectory(
        new Pose2d(27.4 * mpi, 15.4 * mpi, new Rotation2d(Math.toRadians(0))),
        new Pose2d(11 * mpi, 2 * mpi, new Rotation2d(Math.toRadians(0))),
        new Translation2d(25.3 * mpi, 10.42 * mpi), 
        new Translation2d(23.08 * mpi, 5.07 * mpi),
        new Translation2d(16.52 * mpi, 1.65 * mpi)
    ); 

    public static Trajectory flowerFour = generateTrajectory(
        new Pose2d(11 * mpi, 2 * mpi, new Rotation2d(Math.toRadians(0))),
        new Pose2d(0, 20 * mpi, new Rotation2d(Math.toRadians(0))),
        new Translation2d(6.22 * mpi, 5.18 * mpi), 
        new Translation2d(4.58 * mpi, 10.44 * mpi),
        new Translation2d(2.69 * mpi, 16.24 * mpi)
    ); 

    public static Trajectory nada = generateTrajectory(
        new Pose2d(),
        new Pose2d(),
        new Translation2d(),
        new Translation2d(),
        new Translation2d()
    ); 

    public static Trajectory generateTrajectory(Pose2d starting, Pose2d ending, Translation2d interior1, Translation2d interior2, Translation2d interior3){
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

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            starting,
            interiorWaypoints, 
            ending,
            config
        );
        System.out.println("End config pose: " + Robot.m_drivetrainSubsystem.getMPoseX());
        return trajectory;
    }
    //the point of the method is to not have to type out new pose2d etc etc every time you want a trajectory
    //s stands for start (as in the starting pose2d), e is for end, and wp1,2, and 3 are waypoints
    // public static Trajectory easyTrajectory(double sx, double sy, double sr, double ex, double ey, double er, double wp1x, double wp1y, double wp2x, double wp2y, double wp3x, double wp3y){
    //     Trajectory traj = generateTrajectory(
    //         new Pose2d(sx * mpi, sy * mpi, new Rotation2d(Math.toRadians(sr))),
    //         new Pose2d(ex * mpi, ey * mpi, new Rotation2d(Math.toRadians(er))),
    //         new Translation2d(wp1x * mpi, wp1y * mpi), 
    //         new Translation2d(wp2x * mpi, wp2y * mpi),
    //         new Translation2d(wp3x * mpi, wp3y * mpi)
    //     );
    //     return traj;
    // };
    //new version with array
    public static Trajectory easyTrajectory(double[] coords){
        Trajectory traj = generateTrajectory(
            new Pose2d(coords[0] * mpi, coords[1] * mpi, new Rotation2d(Math.toRadians(coords[2]))),
            new Pose2d(coords[3] * mpi, coords[4] * mpi, new Rotation2d(Math.toRadians(coords[5]))),
            new Translation2d(coords[6] * mpi, coords[7] * mpi), 
            new Translation2d(coords[8] * mpi, coords[9] * mpi),
            new Translation2d(coords[10] * mpi, coords[10] * mpi)
        );
        return traj;
    }
    //avery says you need to do it this way, you can't just call easyTrajectory
    static double[] ntc = {2,3,4,5,6,7,8,9,10,11,12};
    public static Trajectory ntrajectory = easyTrajectory(ntc);
    //public static Trajectory ntrajectory = easyTrajectory(1,2,3,4,5,6,7,8,9,10,11,12);

    public static AutonomousBaseMP easyTPath(double[] tp1, double[] tp2, double[] tp3, double[] tp4){
        Trajectory trajectory1 = easyTrajectory(tp1);
        Trajectory trajectory2 = easyTrajectory(tp2);
        Trajectory trajectory3 = easyTrajectory(tp3);
        Trajectory trajectory4 = easyTrajectory(tp4);
       
        return new AutonomousBaseMP(
            trajectory1,
            trajectory2,
            trajectory3,
            trajectory4
        );
    }
    


}