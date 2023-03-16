package frc.robot.subsystems.Vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousBasePD;
import frc.robot.autonomous.StateWithCoordinate;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.subsystems.*;
import frc.robot.Buttons;

public class AprilTagSubsystem {
    public static enum AprilTagSequence{
        DETECT,
        CORRECTPOSITION,
        OFF;
    }

    public static AprilTagSequence states = AprilTagSequence.OFF; 
    private static DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

    public void setState(AprilTagSequence newState){
        states = newState;
    }
    
    private static LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
    private static AutonomousBasePD autonomousBasePD;

    public void init(){
        limeLightSubsystem.setPipeline(0.0);
        //autonomousBasePD.resetControllers();
        //Robot.m_drivetrainSubsystem.resetOdometry(new Pose2d(AprilTagLocation.scoringPoses[4].getX() -36.5*Constants.METERS_PER_INCH, AprilTagLocation.scoringPoses[4].getY() - 12.0*Constants.METERS_PER_INCH, new Rotation2d(Math.toRadians(0.0))));
        //System.out.println("resetted odometry in INIT to: " + DrivetrainSubsystem.m_pose);
        setState(AprilTagSequence.OFF);
    }
    
    public void periodic(){
        System.out.println(limeLightSubsystem.getTv() + "  state: " + states);
        if(states == AprilTagSequence.DETECT){
            limeLightSubsystem.reset();
            // System.out.println("detect");
            if(LimeLightSubsystem.tv!=0){ //made private in limelightss but changed due to pull request- ask katherine!
                System.out.println("APRIL TAG DETECTED!!!!!!");
                setState(AprilTagSequence.CORRECTPOSITION);
                //autonomousBasePD.resetControllers();
                autonomousBasePD = new AutonomousBasePD(DrivetrainSubsystem.m_pose, new StateWithCoordinate[]{                    
                    new StateWithCoordinate(AutoStates.FIRST),
                    new StateWithCoordinate(AutoStates.DRIVE, AprilTagLocation.scoringPoses[Buttons.scoringCol]),
                    //new StateWithCoordinate(Buttons.level)
                });
                System.out.println("Setpoint: " + AprilTagLocation.scoringPoses[Buttons.scoringCol]);
                autonomousBasePD.init();
                System.out.println("Autonomous Base PD: " + autonomousBasePD);
                }
        } else if(states == AprilTagSequence.CORRECTPOSITION){
            System.out.println("Limelight tv: " + limeLightSubsystem.getTv());
        
            if(limeLightSubsystem.getTv() != 0.0){
                double[] tempArray = limeLightSubsystem.networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
                drivetrainSubsystem.resetOdometry(new Pose2d(tempArray[0], tempArray[1], new Rotation2d (Math.toRadians(tempArray[5]))));
                System.out.println("Reset odometry to this m_pose: " + DrivetrainSubsystem.m_pose);
                autonomousBasePD.periodic();

                /*if(autonomousBasePD.xAtSetpoint() && autonomousBasePD.yAtSetpoint() && autonomousBasePD.turnAtSetpoint()){
                    setState(AprilTagSequence.OFF);
                    Robot.m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, Robot.m_drivetrainSubsystem.getPoseRotation()));
                    System.out.println("finished correcting position!!!!!");
                }*/    
            }else{
                System.out.println("don't see april tag");
                setState(AprilTagSequence.DETECT);
            }
            
        } else if(states == AprilTagSequence.OFF){
           // System.out.println("in off");

        }       
    }
}