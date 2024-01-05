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
    //lauren + caro note: copied and pasted from Paths.java (for HDLEAVEBMP)
    //Avery note: we might need to adjust for origin in the middle
    private static final double STARTING_X = 0.0;
    private static final double ENDING_X = 100; 
    //frequently used midpoints (or never used midpoints)
    private static final double HB_Y_B = 200.046; 
    private static final double HD_Y_B = 54.69;
    private static final double HB_Y_R = 200.046;
    private static final double HD_Y_R = 54.69;
//Write methods for each trajectory - 11/17/2023
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
//NADA IS EXTREMELY BROKEN DO NOT USE - 11/17/2023
    /*public static Trajectory nada = generateTrajectory(
        new Pose2d(),
        new Pose2d(),
        new Translation2d(),
        new Translation2d(),
        new Translation2d()
    ); */
//TODO: EDIT HDMP AND HBMP - 11/17/2023
    public static Trajectory HDMP = generateTrajectory( //for all HD paths
        new Pose2d(), //new Pose2d(STARTING_X * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(60 * mpi, 0 * mpi, new Rotation2d(Math.toRadians(180.0))),//ENDING_X * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))),
        new Translation2d(15 * mpi, 0 * mpi),
        new Translation2d(30 * mpi, 0 * mpi),
        new Translation2d(45 * mpi, 0 * mpi)
    );

    /* TODO: ERROR TOO CLOSE TOGETHER. 

    public static Trajectory HBMP = generateTrajectory( //for all HB paths
        new Pose2d(STARTING_X * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(ENDING_X * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))),
        new Translation2d(75 * mpi, HB_Y_B * mpi),
        new Translation2d(85 * mpi, HB_Y_B * mpi),
        new Translation2d(95 * mpi, HB_Y_B * mpi)
    );
    */

    //shouldn't be complaining about this one, what is it 11/17/. 
    //are they too close? issue with directionality? or units. clockwise positive angles. 
    public static Trajectory ENGAGEMP = generateTrajectory( //all measurements in meters already! PathPlanner
        new Pose2d(1.95, 2.77, new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(4.12, 2.73, new Rotation2d(Math.toRadians(180.0))),
        new Translation2d(2.44, 4.31),
        new Translation2d(3.79, 4.79),
        new Translation2d(5.71, 3.52)
    );
   

    public static Trajectory generateTrajectory(Pose2d starting, Pose2d ending, Translation2d interior1, Translation2d interior2, Translation2d interior3){
        System.out.println("Into generate trajectory"); 
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(interior1);
        interiorWaypoints.add(interior2);
        interiorWaypoints.add(interior3);

        SwerveDriveKinematicsConstraint swerveDriveKinematicsConstraint = new SwerveDriveKinematicsConstraint(DrivetrainSubsystem.getMKinematics(), DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
        MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*.75);
        System.out.println("Made constraints!");

        TrajectoryConfig config = new TrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 1); //4.96, 2.8 //we should maybe look into this further
        System.out.println("Start config pose: " + Robot.m_drivetrainSubsystem.getMPoseX());
        config.addConstraint(swerveDriveKinematicsConstraint);
        config.addConstraint(maxVelocityConstraint);
        // look into using traj configs. its possiblwe are not using/;applying it correctly
        //our way of generating trajectory. It is BROKEN 
        System.out.println("to generate trajectory WPIlib"); 
        //Avery Note: this line (below) is the broken one! (11/3/23)
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            starting,
            interiorWaypoints, 
            ending,
            config
        );
        System.out.println("Generated trajectory! " + trajectory.getTotalTimeSeconds());
        return trajectory;
    }
}