package frc.robot.subsystems;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Quaternion;
//import java.util.Arrays; 

public class AprilTagLocation{
    /*public static final Pose3d aprilTagPoses[] = {
        new Pose3d(new Translation3d(15.513558, 1.071626, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))),
        new Pose3d(new Translation3d(15.513558, 2.748026, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))),
        new Pose3d(new Translation3d(15.513558, 4.424426, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))),
        new Pose3d(new Translation3d(16.178784, 6.749796, 0.695452), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))),
        new Pose3d(new Translation3d(0.36195, 6.749796, 0.695452), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))),
        new Pose3d(new Translation3d(1.02743, 4.424426, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))),
        new Pose3d(new Translation3d(1.02743, 2.748026, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))),
        new Pose3d(new Translation3d(1.02743, 1.071626, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))}; 
        */

    public static final Pose3d aprilTagPoses[] = {
        new Pose3d(new Translation3d(610.77, 42.19, 18.22), new Rotation3d(0.0, 0.0, 180.0)),
        new Pose3d(new Translation3d(610.77, 108.19, 18.22), new Rotation3d(0.0, 0.0, 180.0)),
        new Pose3d(new Translation3d(610.77, 174.19, 18.22), new Rotation3d(0.0, 0.0, 180.0)),
        new Pose3d(new Translation3d(636.96, 265.74, 27.38), new Rotation3d(0.0, 0.0, 180.0)),
        new Pose3d(new Translation3d(14.25, 265.74, 27.38), new Rotation3d(0.0, 0.0, 0.0)),
        new Pose3d(new Translation3d(40.45, 174.19, 18.22), new Rotation3d(0.0, 0.0, 0.0)),
        new Pose3d(new Translation3d(40.45, 108.19, 18.22), new Rotation3d(0.0, 0.0, 0.0)),
        new Pose3d(new Translation3d(40.45, 42.19, 18.22), new Rotation3d(0.0, 0.0, 0.0))}; 

    public static final Pose2d scoringPoses[] = {
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 20.0), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 42.19), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 64), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 86), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 108.19), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 130), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 152), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 174.19), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(610.77-16-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 196), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(54.25+Constants.BUMPER_WIDTH+ Constants.DRIVETRAIN_WIDTH, 196), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(54.25+ Constants.BUMPER_WIDTH + Constants.DRIVETRAIN_WIDTH, 174.19), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(54.25+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 152), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(54.25+Constants.BUMPER_WIDTH+Constants.DRIVETRAIN_WIDTH, 130), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(54.25+Constants.BUMPER_WIDTH + Constants.DRIVETRAIN_WIDTH, 108.19), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(54.25+ Constants.BUMPER_WIDTH+Constants.DRIVETRAIN_WIDTH, 86), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(54.25+Constants.BUMPER_WIDTH+ Constants.DRIVETRAIN_WIDTH, 64), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(54.25+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 42.19), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(54.25+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 20), new Rotation2d(0.0))
    };
}