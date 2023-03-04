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
        new Pose3d(new Translation3d(15.514, 1.072, 0.463), new Rotation3d(0.0, 0.0, 180.0)),
        new Pose3d(new Translation3d(15.514, 2.748, 0.463), new Rotation3d(0.0, 0.0, 180.0)),
        new Pose3d(new Translation3d(15.514, 4.424, 0.463), new Rotation3d(0.0, 0.0, 180.0)),
        new Pose3d(new Translation3d(16.179, 6.75, 0.695), new Rotation3d(0.0, 0.0, 180.0)),
        new Pose3d(new Translation3d(0.362, 6.75, 0.695), new Rotation3d(0.0, 0.0, 0.0)),
        new Pose3d(new Translation3d(1.027, 4.424, 0.463), new Rotation3d(0.0, 0.0, 0.0)),
        new Pose3d(new Translation3d(1.027, 2.748, 0.463), new Rotation3d(0.0, 0.0, 0.0)),
        new Pose3d(new Translation3d(1.027, 1.072, 0.463), new Rotation3d(0.0, 0.0, 0.0))}; 

    public static final Pose2d scoringPoses[] = {
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 0.508), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 1.072), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 1.626), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 2.184), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 2.748), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 3.302), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 3.861), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 4.424), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 4.978), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH+ Constants.DRIVETRAIN_WIDTH, 4.978), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(1.378+ Constants.BUMPER_WIDTH + Constants.DRIVETRAIN_WIDTH, 4.424), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 3.861), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH+Constants.DRIVETRAIN_WIDTH, 3.302), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH + Constants.DRIVETRAIN_WIDTH, 2.748), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(1.378+ Constants.BUMPER_WIDTH+Constants.DRIVETRAIN_WIDTH, 2.184), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH+ Constants.DRIVETRAIN_WIDTH, 1.626), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 1.072), new Rotation2d(0.0)),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 0.508), new Rotation2d(0.0))
    };
}