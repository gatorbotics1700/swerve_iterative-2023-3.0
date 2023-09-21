package frc.robot.autonomous;
import frc.robot.Constants;
import frc.robot.autonomous.*;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.autonomous.AutonomousBaseEngage;

import javax.swing.plaf.nimbus.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.AutonomousBaseMP;
import frc.robot.autonomous.Trajectories;
import edu.wpi.first.math.trajectory.Trajectory;



public class Paths {
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
        LOWHDPLACELEAVEB,
        LOWHBPLACELEAVEB,
        MIDHDPLACELEAVEB,
        MIDHBPLACELEAVEB,
        LOW_OVER_ENGAGE,
        MID_OVER_ENGAGE,
        TIMED,
        MIDTIMEDENGAGED,
        LOWTIMEDENGAGED,
        DRIVETIMEDENGAGED,
        MP, 
        MP_HD3SCORER, 
        MP_TESTPATH;
    }
    
    // //all of the trajectories for each path go into an array
    // //public static Trajectory[] mp = {Trajectories.uno, Trajectories.dos, Trajectories.tres, Trajectories.nada};
    // public static Trajectory[] mp_hd3scorer = {Trajectories.oneHD3R, Trajectories.twoHD3R, Trajectories.threeHD3R, Trajectories.fourHD3R};
    // public static Trajectory[] mp_testpath = {Trajectories.flowerOne, Trajectories.flowerTwo, Trajectories.flowerThree, Trajectories.flowerFour};

    // //this way you don't need to type out the return new etc etc in each if statement
    // public static AutonomousBaseMP autobasempgenerator(Trajectory[] t){
    //     return new AutonomousBaseMP(t[0], t[1], t[2], t[3]);
    // }
    // public static double[] mp = {};


    //shorter and hopefully easier to use than the previous version
    public static AutonomousBase constructAuto(AUTO_OPTIONS selectedAuto){
        if(selectedAuto == AUTO_OPTIONS.MP){
            //this looks like it might be in meters, but it should be in inches
            return Trajectories.easyTPath(0.0, 0.0, 0.0, 2.0, 0.0, 90.0, 1.0, 0.0, 1.2, 0.25, 1.4, 0.0,
            0.0, 0.0, 0.0, 1.5, 2.3, 0.0, 0.2, 0.5, 0.9, 1.5, 1.5, 2.0,
            0.0, 0.0, 0.0, 2.0, -1.0, 0.0, 0.2, -0.4, 0.9, -0.6, 1.5, -0.8,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        
        }else if(selectedAuto == AUTO_OPTIONS.MP_HD3SCORER){
            return Trajectories.easyTPath(595.614, 20.19, 180, 372.684, 37.193, 0, 520.0, 25.2, 445.0, 30.2, 400.0, 33.19,
            372.60, 37.193, 0.0, 595.461, 43.068, 180.0, 400.0, 33.19, 445.0, 30.2, 520.0, 25.2,
            595.461, 43.068, 180.0, 372.606, 85.622, 0.0, 525.0, 43.5, 454.199, 45.934, 400.0, 55.0,
            372.606, 85.622, 0.0, 595.529, 66.117, 180.0, 400.0, 55.0, 454.199, 45.934, 525.0, 43.5);
        //and then add more else ifs for the other auto options
    
        }else if(selectedAuto== AUTO_OPTIONS.LOWTIMEDENGAGED){
            return new AutonomousBaseEngage(1);
        } else if (selectedAuto== AUTO_OPTIONS.MIDTIMEDENGAGED){
            return new AutonomousBaseEngage(2);
        } else if (selectedAuto == AUTO_OPTIONS.DRIVETIMEDENGAGED){
            return new AutonomousBaseEngage(0);
        }
        else {
            return new AutonomousBaseTimed();
        }
        // if(selectedAuto == AUTO_OPTIONS.TESTPATH){
        //     return new AutonomousBasePD(
        //         new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //         new StateWithCoordinate(AutoStates.FIRST),
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(5 * mpi, -40 * mpi, new Rotation2d(Math.toRadians(180)))), 
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(40 * mpi, 0, new Rotation2d(0))), 
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(0, 30 * mpi, new Rotation2d(0))), 
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(40 * mpi, 30 * mpi, new Rotation2d(0))), 
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(0, 0, new Rotation2d(0))), 
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(20 * mpi, 20 * mpi, new Rotation2d(0))),
        //         new StateWithCoordinate(AutoStates.STOP)

        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.NOGO){
        //     return new AutonomousBasePD(
        //         new Pose2d(STARTING_X, 0.0, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //         new StateWithCoordinate(AutoStates.FIRST),
        //         new StateWithCoordinate(AutoStates.LOWNODE),
        //         new StateWithCoordinate(AutoStates.STOP)
        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.HDLEAVEB || selectedAuto == AUTO_OPTIONS.HBLEAVEB){
        //     return new AutonomousBasePD( 
        //         new Pose2d(STARTING_X * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //             new StateWithCoordinate(AutoStates.FIRST),
        //             new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(225 * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),//278.95
        //             new StateWithCoordinate(AutoStates.STOP) 
        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.HBLEAVEB){
        //     return new AutonomousBasePD(
        //         new Pose2d(STARTING_X * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //         new StateWithCoordinate(AutoStates.FIRST),
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(225 * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
        //         new StateWithCoordinate(AutoStates.STOP)
        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.LOWHDPLACELEAVEB){
        //     return new AutonomousBasePD(
        //         new Pose2d(STARTING_X * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //         new StateWithCoordinate(AutoStates.FIRST),
        //         new StateWithCoordinate(AutoStates.LOWNODE),
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d((225 + 30) * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
        //         new StateWithCoordinate(AutoStates.STOP)
        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.MIDHDPLACELEAVEB){
        //     return new AutonomousBasePD(
        //         new Pose2d(STARTING_X * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //         new StateWithCoordinate(AutoStates.FIRST),
        //         new StateWithCoordinate(AutoStates.MIDNODE),
        //         new StateWithCoordinate(AutoStates.DRIVE, new Pose2d((225 + 30) * mpi, HB_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
        //         new StateWithCoordinate(AutoStates.STOP)
        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.LOWHBPLACELEAVEB){
        //     return new AutonomousBasePD(
        //         new Pose2d(STARTING_X * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //             new StateWithCoordinate(AutoStates.FIRST),
        //             new StateWithCoordinate(AutoStates.LOWNODE),
        //             new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(225 * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
        //             new StateWithCoordinate(AutoStates.STOP)
        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.MIDHBPLACELEAVEB){
        //     return new AutonomousBasePD(
        //         new Pose2d(STARTING_X * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //             new StateWithCoordinate(AutoStates.FIRST),
        //             new StateWithCoordinate(AutoStates.MIDNODE),
        //             new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(225 * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
        //             new StateWithCoordinate(AutoStates.STOP)
        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.LOW_OVER_ENGAGE){
        //     return new AutonomousBasePD(
        //         new Pose2d(STARTING_X * mpi, HD_Y_B * mpi + 4*mpi, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //             new StateWithCoordinate(AutoStates.FIRST),
        //             new StateWithCoordinate(AutoStates.LOWNODE),
        //             new StateWithCoordinate(AutoStates.FASTDRIVE, new Pose2d(265 * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
        //             new StateWithCoordinate(AutoStates.ENGAGE),
        //             new StateWithCoordinate(AutoStates.STOP)
        //         }
        //     );
        // } else if (selectedAuto == AUTO_OPTIONS.MID_OVER_ENGAGE){
        //     return new AutonomousBasePD(
        //         new Pose2d(STARTING_X * mpi, HD_Y_B * mpi + 4*mpi, new Rotation2d(Math.toRadians(180.0))), 
        //         new StateWithCoordinate[]{
        //             new StateWithCoordinate(AutoStates.FIRST),
        //             new StateWithCoordinate(AutoStates.MIDNODE),
        //             new StateWithCoordinate(AutoStates.FASTDRIVE, new Pose2d(265 * mpi, HD_Y_B * mpi, new Rotation2d(Math.toRadians(180.0)))),
        //             new StateWithCoordinate(AutoStates.ENGAGE),
        //             new StateWithCoordinate(AutoStates.STOP)
        //         }
        //     );
       // } else if(selectedAuto == AUTO_OPTIONS.MP){
        
        

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
