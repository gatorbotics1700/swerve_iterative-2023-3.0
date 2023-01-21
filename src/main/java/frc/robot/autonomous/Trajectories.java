package frc.robot.autonomous;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Trajectories{

    public static Trajectory uno = AutonomousBaseMP.generateTrajectory(
        new Pose2d(), 
        new Pose2d(20, 25, new Rotation2d(0)), 
        new Translation2d(3, 5), 
        new Translation2d(9, 17), 
        new Translation2d(15, 20)
    );

    public static Trajectory dos = AutonomousBaseMP.generateTrajectory(
        new Pose2d(), 
        new Pose2d(15, 23, new Rotation2d(0)), 
        new Translation2d(2, 5), 
        new Translation2d(9, 15), 
        new Translation2d(15, 20)
    );

    public static Trajectory tres = AutonomousBaseMP.generateTrajectory(
        new Pose2d(), 
        new Pose2d(20, -10, new Rotation2d(0)), 
        new Translation2d(2, -4), 
        new Translation2d(9, -6), 
        new Translation2d(15, -8)
    );

    
}