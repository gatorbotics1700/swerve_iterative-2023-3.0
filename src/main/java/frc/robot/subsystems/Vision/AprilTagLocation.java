package frc.robot.subsystems.Vision;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class AprilTagLocation{
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
        //mid+high scoring
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 0.508), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 1.072), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 1.626), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 2.184 + 0.0508), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 2.748), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 3.302), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 3.861), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 4.424), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 4.978), new Rotation2d(Math.toRadians(0.0))),
        
        //low row scoring
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 0.508), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 1.072), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 1.626), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 2.184 + 0.0508), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 2.748), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 3.302), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 3.861), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 4.424), new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(new Translation2d(15.107-0.3048-Constants.BUMPER_WIDTH-Constants.DRIVETRAIN_WIDTH, 4.978), new Rotation2d(Math.toRadians(0.0))),
        
        //mid+high scoring
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH+ Constants.DRIVETRAIN_WIDTH, 4.978), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+ Constants.BUMPER_WIDTH + Constants.DRIVETRAIN_WIDTH, 4.424), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 3.861), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH+Constants.DRIVETRAIN_WIDTH, 3.302), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH + Constants.DRIVETRAIN_WIDTH, 2.748), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+ Constants.BUMPER_WIDTH+Constants.DRIVETRAIN_WIDTH, 2.184), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH+ Constants.DRIVETRAIN_WIDTH, 1.626), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 1.072), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 0.508), new Rotation2d(Math.toRadians(180.0))),
        
        //low row scoring 
        new Pose2d(new Translation2d(1.378-0.3048+Constants.BUMPER_WIDTH+ Constants.DRIVETRAIN_WIDTH, 4.978), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378-0.3048+ Constants.BUMPER_WIDTH + Constants.DRIVETRAIN_WIDTH, 4.424), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378-0.3048+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 3.861), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378-0.3048+Constants.BUMPER_WIDTH+Constants.DRIVETRAIN_WIDTH, 3.302), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378-0.3048+Constants.BUMPER_WIDTH + Constants.DRIVETRAIN_WIDTH, 2.748), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378-0.3048+ Constants.BUMPER_WIDTH+Constants.DRIVETRAIN_WIDTH, 2.184), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378-0.3048+Constants.BUMPER_WIDTH+ Constants.DRIVETRAIN_WIDTH, 1.626), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378-0.3048+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 1.072), new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(new Translation2d(1.378-0.3048+Constants.BUMPER_WIDTH +Constants.DRIVETRAIN_WIDTH, 0.508), new Rotation2d(Math.toRadians(180.0)))
    };
}