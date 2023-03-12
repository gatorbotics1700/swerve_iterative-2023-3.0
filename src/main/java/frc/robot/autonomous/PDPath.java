package frc.robot.autonomous;
import frc.robot.Constants;
import frc.robot.autonomous.*;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;

import javax.swing.plaf.nimbus.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class PDPath {
    private static double mpi = Constants.METERS_PER_INCH;

    public AutonomousBasePD testPath = new AutonomousBasePD(
        new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180.0))), 
        new StateWithCoordinate[]{
          new StateWithCoordinate(AutoStates.FIRST),
          new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(5 * mpi, -40 * mpi, new Rotation2d(Math.toRadians(180)))), 
          new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(40 * mpi, 0, new Rotation2d(0))), 
          new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(0, 30 * mpi, new Rotation2d(0))), 
          new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(40 * mpi, 30 * mpi, new Rotation2d(0))), 
          new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(0, 0, new Rotation2d(0))), 
          new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(20 * mpi, 20 * mpi, new Rotation2d(0))),
          new StateWithCoordinate(AutoStates.STOP)

        }
    );

    public static AutonomousBasePD noGoR = new AutonomousBasePD(
        new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))), 
        new StateWithCoordinate[]{
          new StateWithCoordinate(AutoStates.FIRST),  
          new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
          new StateWithCoordinate(AutoStates.STOP)
        }
    );
    
    public static AutonomousBasePD noGoB = new AutonomousBasePD(
        new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180.0))), 
        new StateWithCoordinate[]{
          new StateWithCoordinate(AutoStates.FIRST),
          new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
          new StateWithCoordinate(AutoStates.STOP)
        }
    );

    public static AutonomousBasePD HBLeaveB = new AutonomousBasePD(
        new Pose2d(56.069 * mpi, 200.046 * mpi, new Rotation2d(Math.toRadians(180.0))), 
        new StateWithCoordinate[]{
        new StateWithCoordinate(AutoStates.FIRST),
        new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(219.915 * mpi, 200.046 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.STOP)
        }
    );

    public static AutonomousBasePD HBLeaveR = new AutonomousBasePD(
        new Pose2d(56.069 * mpi, 200.046 * mpi, new Rotation2d(Math.toRadians(0.0))), 
        new StateWithCoordinate[]{
            new StateWithCoordinate(AutoStates.FIRST),
            new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
            new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(219.915 * mpi, 200.046 * mpi, new Rotation2d(Math.toRadians(0.0)))),
            new StateWithCoordinate(AutoStates.STOP)
        }
    );

    //hd leave are changed
    public static AutonomousBasePD HDLeaveB = new AutonomousBasePD(
        new Pose2d(20.19 * mpi, 56.069 * mpi, new Rotation2d(Math.toRadians(180.0))), 
        new StateWithCoordinate[]{
            new StateWithCoordinate(AutoStates.FIRST),
            new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
            new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(219.915 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(180.0)))),
            new StateWithCoordinate(AutoStates.STOP)
        }
    );

    public static AutonomousBasePD HDLeaveR = new AutonomousBasePD(
        new Pose2d( 595.641 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(180.0))), //ROTATION MUST BE CHANGED BACK
        new StateWithCoordinate[]{
            new StateWithCoordinate(AutoStates.FIRST),
            new StateWithCoordinate(AutoStates.MIDNODE),
            new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(431.768 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(0.0)))),
            new StateWithCoordinate(AutoStates.STOP)
        }
    );

    public static AutonomousBasePD engageChargeB = new AutonomousBasePD(
        new Pose2d(56.069 * mpi, 108.015* mpi, new Rotation2d(Math.toRadians(180.0))), 
        new StateWithCoordinate[]{
        new StateWithCoordinate(AutoStates.FIRST),
        new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(152.812 * mpi, 108.015 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.BALANCING)
        }
    );

    public static AutonomousBasePD engageChargeR = new AutonomousBasePD(
        new Pose2d(56.069 * mpi, 108.015* mpi, new Rotation2d(Math.toRadians(0.0))), 
        new StateWithCoordinate[]{
            new StateWithCoordinate(AutoStates.FIRST),
            new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
            new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(152.812 * mpi, 108.015 * mpi, new Rotation2d(Math.toRadians(0.0)))),
            new StateWithCoordinate(AutoStates.BALANCING)
        }
    );

    public static AutonomousBasePD HDIntakeEngageB = new AutonomousBasePD(
        new Pose2d(595.614 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(180.0))), 
        new StateWithCoordinate[]{
        new StateWithCoordinate(AutoStates.FIRST),
        new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(372.684 * mpi, 36.19 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(394.033 * mpi, 64.004 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(494.824 * mpi, 83.368 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.BALANCING)
        }
    );

    public static AutonomousBasePD HDIntakeEngageR = new AutonomousBasePD(
        new Pose2d(595.614 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(0.0))), 
        new StateWithCoordinate[]{
        new StateWithCoordinate(AutoStates.FIRST),
        new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(372.684 * mpi, 36.19 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(394.033 * mpi, 64.004 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(494.824 * mpi, 83.368 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.BALANCING)
        }
    );

    private AutonomousBasePD HD3ScoreR = new AutonomousBasePD(
        new Pose2d(595.614 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(0.0))), 
        new StateWithCoordinate[]{
        new StateWithCoordinate(AutoStates.FIRST),
        new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(372.684 * mpi, 37.193 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(595.461 * mpi, 40.068 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.HIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(454.199 * mpi, 45.934 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(372.606 * mpi, 85.622 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(454.199 * mpi, 40.000 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(595.529 * mpi, 66.117 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.STOP)
        }
    );

    private AutonomousBasePD HD3ScoreB = new AutonomousBasePD(
        new Pose2d(595.614 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(180.0))), 
        new StateWithCoordinate[]{
        new StateWithCoordinate(AutoStates.FIRST),
        new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(372.684 * mpi, 37.193 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(595.461 * mpi, 40.068 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.HIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(454.199 * mpi, 45.934 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(372.606 * mpi, 85.622 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(454.199 * mpi, 40.000 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(595.529 * mpi, 66.117 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.STOP)
        }
    );

    private AutonomousBasePD HB3ScoreR = new AutonomousBasePD(
        new Pose2d(595.614 * mpi, 200.046 * mpi, new Rotation2d(Math.toRadians(0.0))), 
        new StateWithCoordinate[]{
        new StateWithCoordinate(AutoStates.FIRST),    
        new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(372.684 * mpi, 180.683 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(595.614 * mpi, 174.725 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.HIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(444.677 * mpi, 133.515 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(373.677 * mpi, 133.515 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(451.131  * mpi,185.151  * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(594.621 * mpi, 154.368 * mpi, new Rotation2d(Math.toRadians(0.0)))),
        new StateWithCoordinate(AutoStates.STOP)
        }
    );

    private AutonomousBasePD HB3ScoreB = new AutonomousBasePD(
        new Pose2d(595.614 * mpi, 200.046 * mpi, new Rotation2d(Math.toRadians(180.0))), 
        new StateWithCoordinate[]{
        new StateWithCoordinate(AutoStates.FIRST),
        new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(372.684 * mpi, 180.683 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(595.614 * mpi, 174.725 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.HIGHNODE),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(444.677 * mpi, 133.515 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(373.677 * mpi, 133.515 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.INTAKING),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(451.131  * mpi,185.151  * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(594.621 * mpi, 154.368 * mpi, new Rotation2d(Math.toRadians(180.0)))),
        new StateWithCoordinate(AutoStates.STOP)
        }
    );

}
