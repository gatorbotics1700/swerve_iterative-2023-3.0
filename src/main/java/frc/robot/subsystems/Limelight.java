package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousBasePD;
import frc.robot.autonomous.StateWithCoordinate;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;

public class Limelight{
    private VisionStates visionState;
    private NetworkTable networkTable;
    private AutonomousBasePD autonomousBasePD;
    private DrivetrainSubsystem drivetrainSubsystem;
    private final double RR = 0.4; //meters
    private final Pose2d intakePoseRed = new Pose2d(15.037816, 0.4191, new Rotation2d(Math.toRadians(270)));//WRONG
    private final Pose2d intakePoseBlue = new Pose2d(15.037816, 7.559802, new Rotation2d(Math.toRadians(90)));//WRONG
    private final Pose2d intakePose3 = new Pose2d(-7.24+RR, 0.42-RR, new Rotation2d(Math.toRadians(180))); //the subtraction might be unfounded

    public enum VisionStates{
        DETECT,
        DRIVE,
        STOP;
    }
    
    public void setState(VisionStates newVisionState){
        visionState = newVisionState;
    }
    
    public Limelight(){
        init();
    }

    public void init(){
        visionState = VisionStates.STOP;
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        setPipeline(0.0);
        autonomousBasePD = new AutonomousBasePD(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new StateWithCoordinate[]{});
        drivetrainSubsystem = Robot.m_drivetrainSubsystem;
    }
    
    public void periodic(){
        //System.out.println("Vision State: " + visionState);
        if(visionState == VisionStates.DETECT){
            if(getTv()==1.0){
                visionState = VisionStates.DRIVE;
                double[] botArray = networkTable.getEntry("botpose").getDoubleArray(new double[6]);
                //tempArray holds [x,y,z,roll,pitch,yaw,latency]
                drivetrainSubsystem.resetOdometry(new Pose2d(botArray[0], botArray[1], new Rotation2d(botArray[5])));
                System.out.println("Setting robot pose to: " + new Pose2d(botArray[0], botArray[1], new Rotation2d(botArray[5])));
                System.out.println("pose: x " + drivetrainSubsystem.getMPoseX() + "y " + drivetrainSubsystem.getMPoseY());
                double id = networkTable.getEntry("tid").getDouble(0.0);
                if(id==6){
                    autonomousBasePD = new AutonomousBasePD(
                        new Pose2d(botArray[0], botArray[1], new Rotation2d(botArray[5])), 
                        new StateWithCoordinate[]{
                            new StateWithCoordinate(AutoStates.FIRST),
                            new StateWithCoordinate(AutoStates.DRIVE, intakePose3), //changed 5/16
                            new StateWithCoordinate(AutoStates.STOP)
                        });  
                }else if(id == 21){
                    autonomousBasePD = new AutonomousBasePD(
                    new Pose2d(0, 0, drivetrainSubsystem.getPoseRotation()), 
                    new StateWithCoordinate[]{
                        new StateWithCoordinate(AutoStates.FIRST),
                        new StateWithCoordinate(AutoStates.DRIVE, intakePoseBlue),
                        new StateWithCoordinate(AutoStates.STOP)
                    });
                }else if(id == 3){
                    autonomousBasePD = new AutonomousBasePD(
                    new Pose2d(0, 0, drivetrainSubsystem.getPoseRotation()), 
                    new StateWithCoordinate[]{
                        new StateWithCoordinate(AutoStates.FIRST),
                        new StateWithCoordinate(AutoStates.DRIVE, intakePose3),
                        new StateWithCoordinate(AutoStates.STOP)
                    });
                }
                autonomousBasePD.init();
            }
        }else if(visionState == VisionStates.DRIVE){
            autonomousBasePD.periodic();
            if(autonomousBasePD.states==AutoStates.STOP){
                visionState = VisionStates.STOP;
            }
        }else{

        }
    }
    
    private void setPipeline(double pipeline){
        networkTable.getEntry("pipeline").setNumber(pipeline);
    }

    private double getTv(){
        return networkTable.getEntry("tv").getDouble(0.0);
    }

    private double getTx(){
        return networkTable.getEntry("tx").getDouble(0.0);
    }

    private double getTy(){
        return networkTable.getEntry("ty").getDouble(0.0);
    }
}