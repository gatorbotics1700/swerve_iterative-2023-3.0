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

    //the point of these two methods is to eliminate the redundancy of typing out all of the Pose2ds everytime you make a new trajectory
    //hopefully they will shorten the code and make it easier to debug
    //s is for start, e is for end, and wp is for waypoint
    public static Trajectory easyTraj(double sx, double sy, double sr, double ex, double ey, double er, double wp1x, double wp1y, double wp2x, double wp2y, double wp3x, double wp3y){
        return generateTrajectory(
            new Pose2d(sx * mpi, sy * mpi, new Rotation2d(Math.toRadians(sr))),
            new Pose2d(ex * mpi, ey * mpi, new Rotation2d(Math.toRadians(er))),
            new Translation2d(wp1x * mpi, wp1y * mpi), 
            new Translation2d(wp2x * mpi, wp2y * mpi),
            new Translation2d(wp3x * mpi, wp3y * mpi)
        );
    }
    //it's a ton of variables instead of an array because this way when you type in the values when you call the method it tells you which value is which
    public static AutonomousBaseMP easyTPath(double sx1, double sy1, double sr1, double ex1, double ey1, double er1, double wp1x1, double wp1y1, double wp2x1, double wp2y1, double wp3x1, double wp3y1,
    double sx2, double sy2, double sr2, double ex2, double ey2, double er2, double wp1x2, double wp1y2, double wp2x2, double wp2y2, double wp3x2, double wp3y2, 
    double sx3, double sy3, double sr3, double ex3, double ey3, double er3, double wp1x3, double wp1y3, double wp2x3, double wp2y3, double wp3x3, double wp3y3,
    double sx4, double sy4, double sr4, double ex4, double ey4, double er4, double wp1x4, double wp1y4, double wp2x4, double wp2y4, double wp3x4, double wp3y4){
        Trajectory traj1 = easyTraj(sx1, sy1, sr1, ex1, ey1, er1, wp1x1, wp1y1, wp2x1, wp2y1, wp3x1, wp3y1);
        Trajectory traj2 = easyTraj(sx2, sy2, sr2, ex2, ey2, er2, wp1x2, wp1y2, wp2x2, wp2y2, wp3x2, wp3y2);
        Trajectory traj3 = easyTraj(sx3, sy3, sr3, ex3, ey3, er3, wp1x3, wp1y3, wp2x3, wp2y3, wp3x3, wp3y3);
        Trajectory traj4 = easyTraj(sx4, sy4, sr4, ex4, ey4, er4, wp1x4, wp1y4, wp2x4, wp2y4, wp3x4, wp3y4);
        return new AutonomousBaseMP(
            traj1,
            traj2,
            traj3,
            traj4
        );
    }
    //the point of the method is to not have to type out new pose2d etc etc every time you want a trajectory
    // //the order of the variables in the array is as follows: starting coords(xyr), end coords, and three waypoint coords(xy)
    // public static Trajectory easyTrajectory(double[] coords){
    //     Trajectory traj = generateTrajectory(
    //         new Pose2d(coords[0] * mpi, coords[1] * mpi, new Rotation2d(Math.toRadians(coords[2]))),
    //         new Pose2d(coords[3] * mpi, coords[4] * mpi, new Rotation2d(Math.toRadians(coords[5]))),
    //         new Translation2d(coords[6] * mpi, coords[7] * mpi), 
    //         new Translation2d(coords[8] * mpi, coords[9] * mpi),
    //         new Translation2d(coords[10] * mpi, coords[10] * mpi)
    //     );
    //     return traj;
    // }
    
    // static double[] ntc = {2,3,4,5,6,7,8,9,10,11,12};
    // //avery says you need to do it this way, you can't just call easyTrajectory
    // public static Trajectory ntrajectory = easyTrajectory(ntc);
    // //public static Trajectory ntrajectory = easyTrajectory(1,2,3,4,5,6,7,8,9,10,11,12);
    
    // //TPath stands for trajectory path, rename it if that isn't intuitive
    // public static AutonomousBaseMP easyTPath(double[] tp1, double[] tp2, double[] tp3, double[] tp4){
    //     Trajectory trajectory1 = easyTrajectory(tp1);
    //     Trajectory trajectory2 = easyTrajectory(tp2);
    //     Trajectory trajectory3 = easyTrajectory(tp3);
    //     Trajectory trajectory4 = easyTrajectory(tp4);
       
    //     return new AutonomousBaseMP(
    //         trajectory1,
    //         trajectory2,
    //         trajectory3,
    //         trajectory4
    //     );
    // }

    
    


}