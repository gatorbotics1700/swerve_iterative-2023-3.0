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
    private final double LLHEIGHT = 1.35;
    private final double ATHEIGHT = 1.15;
    private final double LLANGLE = 0.0;
    private final double RR = 0.3937; //intended to increase confusion- in meters tho!
    private AutonomousBasePD autonomousBasePD;
    private DrivetrainSubsystem drivetrainSubsystem;

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
        System.out.println("Vision State: " + visionState);
        if(visionState == VisionStates.DETECT){
            if(getTv()==1.0){
                visionState = VisionStates.DRIVE;
                double distancey = (ATHEIGHT - LLHEIGHT)/Math.tan(LLANGLE + getTy());
                double distancex = Math.tan(getTx())*distancey;
                System.out.println("Distance Y: " + distancey + " Distance X: " + distancex);
                autonomousBasePD = new AutonomousBasePD(
                    new Pose2d(0, 0, drivetrainSubsystem.getPoseRotation()), 
                    new StateWithCoordinate[]{
                        new StateWithCoordinate(AutoStates.FIRST),
                        new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(distancex, distancey-RR, new Rotation2d(Math.toRadians(180.0)))),
                        new StateWithCoordinate(AutoStates.STOP)
                    });
                autonomousBasePD.init();
            }
        }else if(visionState == VisionStates.DRIVE){
            autonomousBasePD.periodic();
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