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
import frc.robot.subsystems.PneumaticIntakeSubsystem;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import frc.robot.subsystems.Mechanisms;

public class AutonomousBasePD extends AutonomousBase{
    public static final double turnKP= 0.0001; //increased slight *** not tested
    public static final double turnKI= 0.0; 
    public static final double turnKD= 0.0;
    public static final double driveKP= 0.75; //Robot.kP.getDouble(0.00006);//0.00006;
    public static final double driveKI= 0.0; //Robot.kI.getDouble(0.0);//0.0;
    public static final double driveKD= 0.0; //Robot.kD.getDouble(0.0);//0.0;
    private static final double DRIVE_DEADBAND = 2*Constants.METERS_PER_INCH; //meters - previously 3 inches
    private static final double TURN_DEADBAND = 6.0; 

    
    private Pose2d startingCoordinate;
    private StateWithCoordinate[] stateSequence;
    private int i;
    private int intakeCounter = -1; 
    private int highNodeCounter = -1; 
    private double startTimeIntake;
    private double startTimeHigh; 
    
    static DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;
    static Mechanisms mechanisms = Robot.m_mechanisms;
    static PneumaticIntakeSubsystem pneumaticIntakeSubsystem = Robot.m_pneumaticIntakeSubsystem;


    //pids
    private PIDController turnController = new PIDController(turnKP, turnKI, turnKD); 
    private PIDController xController = new PIDController(driveKP, driveKI, driveKD);
    private PIDController yController = new PIDController(driveKP, driveKI, driveKD);
    
    public AutonomousBasePD(Pose2d startingCoordinate, StateWithCoordinate[] stateSequence){
        this.startingCoordinate = startingCoordinate;
        this.stateSequence =  stateSequence;
    }

    public AutonomousBasePD(){ 
    }

    @Override
    public void init(){
        drivetrainSubsystem.resetOdometry(startingCoordinate);
        xController.reset();
        yController.reset();
        turnController.reset();
        turnController.setTolerance(TURN_DEADBAND); 
        xController.setTolerance(DRIVE_DEADBAND);
        yController.setTolerance(DRIVE_DEADBAND);
        i = 0;
        System.out.println("INIT!\nINIT!\nINIT!");

    }

    public void resetControllers(){
        xController.reset();
        yController.reset();
        turnController.reset();
    }

    @Override
    public void periodic()
    {
        AutoStates states = stateSequence[i].state;
        System.out.println("state: " + states);
        if (states == AutoStates.FIRST){
            turnController.setTolerance(TURN_DEADBAND); 
            xController.setTolerance(DRIVE_DEADBAND);
            yController.setTolerance(DRIVE_DEADBAND);
            xController.setSetpoint(0);
            yController.setSetpoint(0);
            turnController.setSetpoint(0);
            i++;  
            System.out.println("moving on to " + stateSequence[i]);
        } else if (states == AutoStates.FIRSTHIGHNODE){ //WE HAVE THIS BECAUSE OF APRILTAGS
            //System.out.println("we've reset to this pose: " + DrivetrainSubsystem.m_pose);
            //if we are done then we need to i++
            i++;  
            System.out.println("moving on to " + stateSequence[i]);
        } else {
            drivetrainSubsystem.drive();
            //System.out.println("pose in auto: " + DrivetrainSubsystem.m_pose.getX()/Constants.METERS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getY()/Constants.METERS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getRotation().getDegrees());
            if (states == AutoStates.DRIVE){
                driveDesiredDistance(stateSequence[i].coordinate);
                if(xController.atSetpoint() && yController.atSetpoint() && turnController.atSetpoint()){
                    i++;  
                    System.out.println("moving on to " + stateSequence[i]);
                }
            }else if(states == AutoStates.HIGHNODE){ 
                //mechanisms.setState(MechanismStates.HIGH_NODE);
                //elevator height, arm length, 0.5 sec then i++ 
                //outtake (from vision)
                //if we are done then we need to i++
                System.out.println("high node");
               if(mechanisms.isDoneHigh()==true){
                   if(highNodeCounter >= -1){
                    startTimeHigh = System.currentTimeMillis();
                   } 
                if(System.currentTimeMillis()-startTimeHigh==0.5){
                    i++;
                    highNodeCounter = -1;
                }
               } 
                System.out.println("moving on to " + stateSequence[i]);
            }else if(states == AutoStates.BALANCING){
                //TODO: make it so the paths that balance end with balancing rather than ending with stop
                //Robot.m_drivetrainSubsystem.pitchBalance(0.0);
            }else if(states == AutoStates.INTAKING){
                if(intakeCounter >= -1){
                    startTimeIntake = System.currentTimeMillis(); 
                }
                if(System.currentTimeMillis()-startTimeIntake==0.5){
                    i++;
                    intakeCounter = -1;
                }
                //pneumaticIntakeSubsystem.setState(PneumaticIntakeStates.ACTUATING); //unclear if we need... based on beam break
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
        turnController.enableContinuousInput(0, 360);
        //System.out.println("xcontroller setpoint: " + xController.getSetpoint());
        //System.out.println("cur pose: " + DrivetrainSubsystem.m_pose);
        //System.out.println("desired pose: " + dPose);
        double speedX = xController.calculate(DrivetrainSubsystem.m_pose.getX(), dPose.getX());
        double speedY = yController.calculate(DrivetrainSubsystem.m_pose.getY(), dPose.getY());
        //System.out.println("m_pose deg: " + DrivetrainSubsystem.m_pose.getRotation().getDegrees() % 360);
        //System.out.println("d_pose deg: " + dPose.getRotation().getDegrees() % 360);
        double speedRotat = turnController.calculate(DrivetrainSubsystem.m_pose.getRotation().getDegrees(), dPose.getRotation().getDegrees());
        //System.out.println("DDDing");
        //System.out.println("speed rotate: " + speedRotat);
        
        if(xAtSetpoint()){
            speedX = 0; 
            //System.out.println("In x deadband.\nX controller error: " + xController.getPositionError() + " in meters.");
        } else {
            speedX = Math.signum(speedX)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(speedX)));  
        }
 
        if(yAtSetpoint()){
            speedY = 0; 
            //System.out.println("In y deadband.");
        } else {
            speedY = Math.signum(speedY)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(speedY)));
        }

        if(turnAtSetpoint()){
            speedRotat = 0;
            //System.out.println("At setpoint");
        } else {
            //System.out.println("Position error: " + turnController.getPositionError());
            speedRotat = Math.signum(speedRotat)*Math.max(Constants.STEER_MOTOR_MIN_VOLTAGE, Math.min(Constants.STEER_MOTOR_MAX_VOLTAGE, Math.abs(speedRotat)));
          //  System.out.println("Speed rotat after: " + speedRotat);
        }

        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRotat, drivetrainSubsystem.getPoseRotation()));  
        double errorX = (dPose.getX() - DrivetrainSubsystem.m_pose.getX());
        double errorY = (dPose.getY() - DrivetrainSubsystem.m_pose.getY());
        double errorRotat = turnController.getPositionError();
        System.out.println("Rotation error: " + errorRotat + " deadband " + turnController.getPositionTolerance());
        System.out.println("Speed X: " + speedX + " Speed Y: " + speedY + " Speed R: " + speedRotat);
        //System.out.println("error:" + errorX + ", " + errorY + ", " + errorRotat);
        //System.out.println("Desired Position: " + dPose.getX() + ", " + dPose.getY());
    }

    @Override
    public void turnDesiredAngle(double desiredTurn){
      //  System.out.println("desired turn: " + desiredTurn);
       // System.out.println("pid val: " + pidturnval);
        //drivetrainSubsystem.setSpeed(
            //ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
            //Math.signum(pidturnval)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE-0.1, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(pidturnval))), 
            //drivetrainSubsystem.getGyroscopeRotation())
        //);
       // System.out.println("error: " + directionController.getPositionError());
    }

    public boolean xAtSetpoint(){
        return Math.abs(xController.getPositionError()) <= DRIVE_DEADBAND;
    }

    public boolean yAtSetpoint(){
        return Math.abs(yController.getPositionError()) <= DRIVE_DEADBAND;
    }

    public boolean turnAtSetpoint(){
        return Math.abs(turnController.getPositionError()) <= TURN_DEADBAND;
    }

}
