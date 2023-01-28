package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Quaternion;
import java.util.Arrays; 

public class AprilTagLocation{
    public static final Pose3d aprilTagPoses[] = [new Pose3d(new Translation3d(15.513558, 1.071626, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))),
    new Pose3d(new Translation3d(15.513558, 2.748026, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))),
    new Pose3d(new Translation3d(15.513558, 4.424426, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))),
    new Pose3d(new Translation3d(16.178784, 6.749796, 0.695452), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))),
    new Pose3d(new Translation3d(0.36195, 6.749796, 0.695452), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))),
    new Pose3d(new Translation3d(1.02743, 4.424426, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))),
    new Pose3d(new Translation3d(1.02743, 2.748026, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))),
    new Pose3d(new Translation3d(1.02743, 1.071626, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))]; //Joanne sorry this does not work yet!

    //Before array
    /*public static final Pose3d aprilTagPoses[] = [new Pose3d(new Translation3d(15.513558, 1.071626, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)));
    public static final Pose3d idTwo = new Pose3d(new Translation3d(15.513558, 2.748026, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)));
    public static final Pose3d idThree = new Pose3d(new Translation3d(15.513558, 4.424426, 0.462788), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)));
    public static final Pose3d idFour = new Pose3d(new Translation3d(16.178784, 6.749796, 0.695452), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)));
    public static final Pose3d idFive = new Pose3d(new Translation3d(0.36195, 6.749796, 0.695452), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)));
    public static final Pose3d idSix = new Pose3d(new Translation3d(1.02743, 4.424426, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)));
    public static final  Pose3d idSeven = new Pose3d(new Translation3d(1.02743, 2.748026, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)),
    public static final Pose3d idEight = new Pose3d(new Translation3d(1.02743, 1.071626, 0.462788), new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))];
 */
}