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
    public static final double turnKP= 0.004;
    public static final double turnKI= 0.0;
    public static final double turnKD= 0.0;
    public static final double driveKP= 0.00004;
    public static final double driveKI= 0.0;
    public static final double driveKD= 0.0;
    private final double DEADBAND = 3;
    private double xdirection;
    private double ydirection;
    private double hypotenuse;

    private Pose2d goalCoordinate1;
    private Pose2d goalCoordinate2; 
    private double turnSetpoint1;
    private double turnSetpoint2; 

    DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

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
        directionController.setTolerance(DEADBAND); 
        distanceController.setTolerance(DEADBAND*Constants.TICKS_PER_INCH);
        directionController.reset();
        distanceController.reset();
        states = States.FIRST;
    }

    public static enum States{
        FIRST,
        DRIVE,
        TURN,
        DRIVE2, 
        TURN2,
        STOP;
    }

    private static States states = States.FIRST;

    public void setState(States newState){
        states = newState;
    }

    @Override
    public void periodic()
    {
        //System.out.println("setpoint: " + driveSetpoint);
        //System.out.println("state: "+states);
        if (states == States.FIRST){
            preDDD(goalCoordinate1); 
            setState(States.DRIVE);
        }
        if (states == States.DRIVE){
            driveDesiredDistance(goalCoordinate1);
            if (distanceController.atSetpoint()){
                preTDA(turnSetpoint1);
                setState(States.STOP);
            }
        } else if(states==States.TURN){
            turnDesiredAngle(turnSetpoint1);
            if(directionController.atSetpoint()){
                setState(States.DRIVE2);
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
        SwerveModulePosition [] positionArray =  new SwerveModulePosition[] {
            drivetrainSubsystem.m_frontLeftModule.getSwerveModulePosition(),
            drivetrainSubsystem.m_frontRightModule.getSwerveModulePosition(),
            drivetrainSubsystem.m_backRightModule.getSwerveModulePosition(),
            drivetrainSubsystem.m_backLeftModule.getSwerveModulePosition() };

        coordinate.resetPosition(drivetrainSubsystem.getGyroscopeRotation(), positionArray, drivetrainSubsystem.getCurrentPose());
        drivetrainSubsystem.getOdometry().resetPosition(drivetrainSubsystem.getGyroscopeRotation(), positionArray, drivetrainSubsystem.getCurrentPose());
        hypotenuse = Math.hypot(coordinate.getX(), coordinate.getY());
        distanceController.reset();
        distanceController.setSetpoint(hypotenuse);
    }

    @Override
    public void driveDesiredDistance(Pose2d coordinate){      
        double speed = distanceController.calculate(Math.sqrt(Math.pow(drivetrainSubsystem.getCurrentPose().getX(), 2)+Math.pow(drivetrainSubsystem.getCurrentPose().getY(), 2)), hypotenuse );
        double directX = coordinate.getX() / Math.sqrt(Math.pow(coordinate.getX(),2) + Math.pow(coordinate.getY(),2));
        double directY = coordinate.getY() / Math.sqrt(Math.pow(coordinate.getX(),2) + Math.pow(coordinate.getY(),2));
        
        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(speed * directX, speed * directY, 0, drivetrainSubsystem.getGyroscopeRotation()));  
        drivetrainSubsystem.drive(); 
    }

    //pre turn desired angle
    public void preTDA(double setpoint){
        directionController.reset();
        directionController.setSetpoint(setpoint);
        drivetrainSubsystem.zeroGyroscope();
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
