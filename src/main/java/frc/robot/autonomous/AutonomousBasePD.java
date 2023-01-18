package frc.robot.autonomous;

import javax.naming.spi.DirStateFactory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class AutonomousBasePD extends AutonomousBase{
    public static final double turnKP= 0.0002;
    public static final double turnKI= 0.0;
    public static final double turnKD= 0.0;
    public static final double driveKP= 0.00004;
    public static final double driveKI= 0.0;
    public static final double driveKD= 0.0;
    private final double DRIVE_DEADBAND = 7;
    private final double TURN_DEADBAND = 0.3;
    private double xdirection;
    private double ydirection;
    private double hypotenuse;

    private Pose2d goalCoordinate1;
    private Pose2d goalCoordinate2; 
    private double turnSetpoint1;
    private double turnSetpoint2; 

    DrivetrainSubsystem drivetrainSubsystem = Robot.getDrivetrainSubsystem();

    //pids
    private PIDController directionController = new PIDController(turnKP, turnKI, turnKD);
    private PIDController distanceController = new PIDController(driveKP, driveKI, driveKD);
    
    public AutonomousBasePD(Pose2d goalCoordinate1, double turnSetpoint1, Pose2d goalCoordinate2, double turnSetpoint2){
        this.goalCoordinate1 = goalCoordinate1;
        this.goalCoordinate2 = goalCoordinate2;
        this.turnSetpoint1 = turnSetpoint1; 
        this.turnSetpoint2 = turnSetpoint2; 
    }

    @Override
    public void init(){
        drivetrainSubsystem.resetOdometry();
        directionController.reset();
        distanceController.reset();
        directionController.setTolerance(TURN_DEADBAND); 
        distanceController.setTolerance(DRIVE_DEADBAND*Constants.TICKS_PER_INCH);
        states = States.TURN;
        System.out.println("INIT!\nINIT!\nINIT!");

    }

    public static enum States{
        FIRST,
        DRIVE,
        TURN,
        DRIVE2, 
        TURN2,
        STOP;
    }

    private static States states = States.TURN;

    public void setState(States newState){
        states = newState;
    }

    @Override
    public void periodic()
    {
       
        //System.out.println("state: "+states);
        if (states == States.FIRST){
            preDDD(goalCoordinate1); 
            System.out.println("we've reset to this pose: " + drivetrainSubsystem.m_pose);
            setState(States.DRIVE);
        }
        if (states == States.DRIVE){
            driveDesiredDistance(goalCoordinate1);
            //System.out.println("inside drive state! pose: " + drivetrainSubsystem.m_pose.getX()/Constants.TICKS_PER_INCH + " " + drivetrainSubsystem.m_pose.getY()/Constants.TICKS_PER_INCH);
            if (distanceController.atSetpoint()){
                preTDA(turnSetpoint1);
                setState(States.TURN);
            }
        } else if(states==States.TURN){
            System.out.println("turning. we are currently at: " + drivetrainSubsystem.getGyroscopeRotation().getDegrees());
            turnDesiredAngle(turnSetpoint1);

            if(directionController.atSetpoint()){
                
                System.out.println("a print statement that says 'we're stopping'");
                setState(States.STOP);
            }
        } else if(states == States.DRIVE2){
            driveDesiredDistance(goalCoordinate2);
            if(distanceController.atSetpoint()){
                preTDA(turnSetpoint2); 
                setState(States.TURN2); 
            }
        } else if(states==States.TURN2){
            turnDesiredAngle(turnSetpoint2);
            if(directionController.atSetpoint()){
                setState(States.STOP); 
            }
        }else{
            drivetrainSubsystem.stopDrive();
        }
    }

    //predrivedesiredistance
    public void preDDD(Pose2d coordinate){
        drivetrainSubsystem.resetOdometry();
        hypotenuse = Math.hypot(coordinate.getX(), coordinate.getY());
        distanceController.setSetpoint(hypotenuse);
    }

    @Override
    public void driveDesiredDistance(Pose2d coordinate){      
        double speed = distanceController.calculate(Math.hypot(drivetrainSubsystem.m_pose.getX(), drivetrainSubsystem.m_pose.getY()), hypotenuse);
        double directX = coordinate.getX() / Math.sqrt(Math.pow(coordinate.getX(),2) + Math.pow(coordinate.getY(),2));
        double directY = coordinate.getY() / Math.sqrt(Math.pow(coordinate.getX(),2) + Math.pow(coordinate.getY(),2));
        
        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(speed * directX, speed * directY, 0, drivetrainSubsystem.getGyroscopeRotation()));  
    }

    //pre turn desired angle
    public void preTDA(double setpoint){
        directionController.reset();
        System.out.println("Preturning");
        directionController.setSetpoint(setpoint);
        drivetrainSubsystem.resetOdometry();
    }

    @Override
    public void turnDesiredAngle(double turnSetpoint){
        directionController.enableContinuousInput(0, 360); //so it goes shortest angle to get to correct
        double pidturnval = directionController.calculate(drivetrainSubsystem.getGyroscopeRotation().getDegrees(), turnSetpoint);
        System.out.println("pid val: " + pidturnval);
        drivetrainSubsystem.setSpeed(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
            Math.signum(pidturnval)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE-0.1, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(pidturnval))), 
            drivetrainSubsystem.getGyroscopeRotation())
        );
        System.out.println("error: " + directionController.getPositionError());
    }

}
