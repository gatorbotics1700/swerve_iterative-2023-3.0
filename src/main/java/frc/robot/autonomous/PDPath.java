package frc.robot.autonomous;
import frc.robot.Constants;
import frc.robot.autonomous.*;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;

import javax.swing.plaf.nimbus.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class PDPath {
    //TODO: FIX ALL THE NUMBERS THESE XS AND YS ARE WRONG ALSO NEED RED ALLIANCE DIFF METHODS
    private static double mpi = Constants.METERS_PER_INCH;
    private static final double STARTING_X = 68.95;
    private static final double HB_Y_B = 200.046;
    private static final double HD_Y_B = 54.69;
    private static final double HB_Y_R = 200.046;
    private static final double HD_Y_R = 54.69;

    public enum AUTO_OPTIONS{
        TESTPATH,
        NOGO,
        HDLEAVEB,
        HBLEAVEB,
        HDPLACELEAVEB,
        HBPLACELEAVEB,
        ENGAGECHARGE,
        TIMED;
    }

    public static AutonomousBase constructAuto(AUTO_OPTIONS selectedAuto){
        if(selectedAuto == AUTO_OPTIONS.TESTPATH){
            return new AutonomousBasePD(
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
        } else if (selectedAuto == AUTO_OPTIONS.NOGO){
            return new AutonomousBasePD(
                new Pose2d(STARTING_X, 0.0, new Rotation2d(Math.toRadians(180.0))), 
                new StateWithCoordinate[]{
                new StateWithCoordinate(AutoStates.FIRST),
                new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
                new StateWithCoordinate(AutoStates.STOP)
                }
            );
        } else if (selectedAuto == AUTO_OPTIONS.HDLEAVEB){
            return new AutonomousBasePD( 
                new Pose2d(STARTING_X * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
                new StateWithCoordinate[]{
                    new StateWithCoordinate(AutoStates.FIRST),
                    new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(225 * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),//278.95
                    new StateWithCoordinate(AutoStates.STOP) 
                }
            );
        } else if (selectedAuto == AUTO_OPTIONS.HBLEAVEB){
            return new AutonomousBasePD(
                new Pose2d(STARTING_X * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
                new StateWithCoordinate[]{
                new StateWithCoordinate(AutoStates.FIRST),
                new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(225 * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
                new StateWithCoordinate(AutoStates.STOP)
                }
            );
        } else if (selectedAuto == AUTO_OPTIONS.HDPLACELEAVEB){
            return new AutonomousBasePD(
                new Pose2d(STARTING_X * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
                new StateWithCoordinate[]{
                new StateWithCoordinate(AutoStates.FIRST),
                new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
                new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(225 * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
                new StateWithCoordinate(AutoStates.STOP)
                }
            );
        } else if (selectedAuto == AUTO_OPTIONS.HBPLACELEAVEB){
            return new AutonomousBasePD(
                new Pose2d(STARTING_X * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
                new StateWithCoordinate[]{
                    new StateWithCoordinate(AutoStates.FIRST),
                    new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
                    new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(225 * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
                    new StateWithCoordinate(AutoStates.STOP)
                }
            );
        } else if (selectedAuto == AUTO_OPTIONS.ENGAGECHARGE){
            return new AutonomousBasePD(
                new Pose2d(STARTING_X * mpi, 108.19* mpi, new Rotation2d(Math.toRadians(180.0))), 
                new StateWithCoordinate[]{
                new StateWithCoordinate(AutoStates.FIRST),
                //new StateWithCoordinate(AutoStates.FIRSTHIGHNODE),
                new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(155.812 * mpi, 108.015 * mpi, new Rotation2d(Math.toRadians(180.0)))),
                new StateWithCoordinate(AutoStates.BALANCING)
                }
            );
        } else { //timed
            return new AutonomousBaseTimed();
        }

        /*public static AutonomousBasePD HDIntakeEngage = new AutonomousBasePD(
            new Pose2d(68.95 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(180.0))), 
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

        public static AutonomousBasePD HD3Score = new AutonomousBasePD(
            new Pose2d(68.95 * mpi, 20.19 * mpi, new Rotation2d(Math.toRadians(180.0))), 
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

        public static AutonomousBasePD HB3Score = new AutonomousBasePD(
            new Pose2d(68.95 * mpi, 200.046 * mpi, new Rotation2d(Math.toRadians(180.0))), 
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
        );*/
    }

}
