package frc.robot.autonomous;

import javax.swing.SwingWorker.StateValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.autonomous.StateWithCoordinate;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;

public class AutonomousBasePD extends AutonomousBase{
    public static final double turnKP= 0.0001; //increased slight *** not tested
    public static final double turnKI= 0.0; 
    public static final double turnKD= 0.0;
    public static final double driveKP= 0.75; //Robot.kP.getDouble(0.00006);//0.00006;
    public static final double driveKI= 0.0; //Robot.kI.getDouble(0.0);//0.0;
    public static final double driveKD= 0.0; //Robot.kD.getDouble(0.0);//0.0;
    private static final double DRIVE_DEADBAND = 1*Constants.METERS_PER_INCH; //meters - previously 3 inches
    private static final double TURN_DEADBAND = 6*Constants.METERS_PER_INCH; 

    
    private static Pose2d startingCoordinate;
    private static StateWithCoordinate[] stateSequence;
    private static int i;
    
    public double desiredTurn;
    
    static DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

    //pids
    private static PIDController directionController = new PIDController(turnKP, turnKI, turnKD);
    private static PIDController turnController = new PIDController(turnKP, turnKI, turnKD); 
    private static PIDController xController = new PIDController(driveKP, driveKI, driveKD);
    private static PIDController yController = new PIDController(driveKP, driveKI, driveKD);


    
    public AutonomousBasePD(Pose2d startingCoordinate, StateWithCoordinate[] stateSequence){
        this.startingCoordinate = startingCoordinate;
        this.stateSequence =  stateSequence;
    }

    public AutonomousBasePD(){ 
    }


    public static void initial(){
        drivetrainSubsystem.resetOdometry(startingCoordinate);
        directionController.reset();
        xController.reset();
        yController.reset();
        turnController.reset();
        //directionController.setTolerance(TURN_DEADBAND);
        turnController.setTolerance(TURN_DEADBAND); 
        xController.setTolerance(DRIVE_DEADBAND);
        yController.setTolerance(DRIVE_DEADBAND);
        i = 0;
        // distanceController.reset(); 
        //distanceController.setTolerance(DRIVE_DEADBAND*Constants.SWERVE_TICKS_PER_INCH);
        System.out.println("INIT!\nINIT!\nINIT!");

    }

    @Override
    public void periodic()
    {
        AutoStates states = stateSequence[i].state;
       // System.out.println("state: " + states);
        if (states == AutoStates.FIRST){
            turnController.setTolerance(TURN_DEADBAND); 
            xController.setTolerance(DRIVE_DEADBAND);
            yController.setTolerance(DRIVE_DEADBAND);
            xController.setSetpoint(0);
            yController.setSetpoint(0);
            turnController.setSetpoint(0);
            i++;
        } else if (states == AutoStates.FIRSTHIGHNODE){
            //System.out.println("we've reset to this pose: " + DrivetrainSubsystem.m_pose);
            //flick intake, elevate, release object
            //if we are done then we need to i++
            i++;
        } else {
            drivetrainSubsystem.drive();
            //System.out.println("pose in auto: " + DrivetrainSubsystem.m_pose.getX()/Constants.METERS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getY()/Constants.METERS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getRotation().getDegrees());
            if (states == AutoStates.DRIVE){
                driveDesiredDistance(stateSequence[i].coordinate);
                if(xController.atSetpoint() && yController.atSetpoint() /*&& turnController.atSetpoint()*/){
                    i++;  
                    System.out.println("moving on");
                }
            }else if(states == AutoStates.HIGHNODE){
                //outtake (from vision)
                //if we are done then we need to i++
                System.out.println("high node");
            }else if(states == AutoStates.BALANCING){
                //pitch pd
            }else if(states == AutoStates.INTAKING){
                //move elevator/intake system (build) and maybe arm pivot?
                //if we are done then we need to i++
            }else{
                drivetrainSubsystem.stopDrive();
            }
        }
    }

    /** 
    @param dTranslation is desired
    */
    @Override
    public void driveDesiredDistance(Pose2d dPose){      
        double speedX = xController.calculate(DrivetrainSubsystem.m_pose.getX(), dPose.getX());
        double speedY = yController.calculate(DrivetrainSubsystem.m_pose.getY(), dPose.getY());
        //System.out.println("m_pose deg: " + DrivetrainSubsystem.m_pose.getRotation().getDegrees() % 360);
        //System.out.println("d_pose deg: " + dPose.getRotation().getDegrees() % 360);
        double speedRotat = turnController.calculate(DrivetrainSubsystem.m_pose.getRotation().getDegrees() % 360, dPose.getRotation().getDegrees() % 360);
        //System.out.println("DDDing");
        //System.out.println("speed rotate: " + speedRotat);
        
        if(xController.atSetpoint()){
            speedX = 0; 
        } else {
            speedX = Math.signum(speedX)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(speedX)));  
        }
 
        if(yController.atSetpoint()){
            speedY = 0; 
        } else {
            speedY = Math.signum(speedY)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(speedY)));
        }

        if(turnController.atSetpoint()){
            speedRotat = 0;
            //System.out.println("At setpoint");
        } else {
            //System.out.println("Position error: " + turnController.getPositionError());
            speedRotat = Math.signum(speedRotat)*Math.max(Constants.STEER_MOTOR_MIN_VOLTAGE, Math.min(Constants.STEER_MOTOR_MAX_VOLTAGE, Math.abs(speedRotat)));
          //  System.out.println("Speed rotat after: " + speedRotat);
        }

        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, 0, drivetrainSubsystem.getPoseRotation()));  
        double errorX = (dPose.getX() - DrivetrainSubsystem.m_pose.getX());
        double errorY = (dPose.getY() - DrivetrainSubsystem.m_pose.getY());
        double errorRotat = (dPose.getRotation().getDegrees() - DrivetrainSubsystem.m_pose.getRotation().getDegrees());
        //System.out.println("Speed X: " + speedX + " Speed Y: " + speedY + " Speed Rotat: " + speedRotat);
        System.out.println("error:" + errorX + ", " + errorY + ", " + errorRotat);
        //System.out.println("Desired Position: " + dPose.getX() + ", " + dPose.getY());
    }

    @Override
    public void turnDesiredAngle(double desiredTurn){
      //  System.out.println("desired turn: " + desiredTurn);
        directionController.enableContinuousInput(0, 360); //so it goes shortest angle to get to correct
        double pidturnval = directionController.calculate(drivetrainSubsystem.getPoseRotation().getDegrees(), desiredTurn);
       // System.out.println("pid val: " + pidturnval);
        drivetrainSubsystem.setSpeed(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
            Math.signum(pidturnval)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE-0.1, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(pidturnval))), 
            drivetrainSubsystem.getGyroscopeRotation())
        );
       // System.out.println("error: " + directionController.getPositionError());
    }

    public boolean xAtSetpoint(){
        return xController.atSetpoint();
    }

    public boolean yAtSetpoint(){
        return yController.atSetpoint();
    }

    public boolean turnAtSetpoint(){
        return turnController.atSetpoint();
    }

}
