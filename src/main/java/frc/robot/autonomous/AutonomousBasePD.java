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
    private final double TURN_DEADBAND = 6;
    private double xdirection;
    private double ydirection;
    private double hypotenuse;

    private Translation2d goalCoordinate1;
    private Translation2d goalCoordinate2; 
    private Translation2d goalCoordinate3; 
    private Translation2d goalCoordinate4; 
    private Translation2d goalCoordinate5;

    private double turnSetpoint1;
    private double turnSetpoint2; 
    private double turnSetpoint3; 
    private double turnSetpoint4; 

    DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

    //pids
    private PIDController directionController = new PIDController(turnKP, turnKI, turnKD);
    private PIDController distanceController = new PIDController(driveKP, driveKI, driveKD);
    
    public AutonomousBasePD(Translation2d goalCoordinate1, Translation2d goalCoordinate2, Translation2d goalCoordinate3, Translation2d goalCoordinate4, Translation2d goalCoordinate5){
        this.goalCoordinate1 = goalCoordinate1;
        this.goalCoordinate2 = goalCoordinate2;
        this.goalCoordinate3 = goalCoordinate3;
        this.goalCoordinate4 = goalCoordinate4;
        this.goalCoordinate5 = goalCoordinate5;

        this.turnSetpoint1 = turnSetpoint1; 
        this.turnSetpoint2 = turnSetpoint2; 
        this.turnSetpoint3 = turnSetpoint3; 
        this.turnSetpoint4 = turnSetpoint4;
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
        DRIVE3,
        TURN3,
        DRIVE4,
        TURN4,
        STOP;
    
    }

    private static States states = States.TURN;

    public void setState(States newState){
        states = newState;
    }
    Translation2d desiredTranslation;
    @Override

    public void periodic()
    {
        
        //System.out.println("state: "+states);
        if (states == States.FIRST){
            desiredTranslation = preDDD(new Translation2d(), goalCoordinate1); 
            System.out.println("we've reset to this pose: " + drivetrainSubsystem.m_pose);
            setState(States.DRIVE);
        }
        if (states == States.DRIVE){
            driveDesiredDistance(desiredTranslation);
            //System.out.println("inside drive state! pose: " + drivetrainSubsystem.m_pose.getX()/Constants.TICKS_PER_INCH + " " + drivetrainSubsystem.m_pose.getY()/Constants.TICKS_PER_INCH);
            if (distanceController.atSetpoint()){
                preTDA(goalCoordinate1, goalCoordinate2);
                setState(States.TURN);
            }
        } else if(states==States.TURN){
            System.out.println("turning. we are currently at: " + drivetrainSubsystem.getGyroscopeRotation().getDegrees());
            turnDesiredAngle(turnSetpoint1);

            if(directionController.atSetpoint()){
                desiredTranslation = preDDD(goalCoordinate1, goalCoordinate2); 
                System.out.println("a print statement that says 'we're stopping'");
                setState(States.STOP);
            }
        } else if(states == States.DRIVE2){
            driveDesiredDistance(desiredTranslation);
            if(distanceController.atSetpoint()){
                preTDA(goalCoordinate2, goalCoordinate3); 
                setState(States.TURN2); 
            }
        } else if(states==States.TURN2){
            turnDesiredAngle(turnSetpoint2);
            if(directionController.atSetpoint()){
                setState(States.STOP); 
            }
        } else if(states == States.DRIVE3){
            driveDesiredDistance(desiredTranslation);
            if(distanceController.atSetpoint()){
                preTDA(goalCoordinate3, goalCoordinate4); 
                setState(States.TURN2); 
            }
        } else if(states==States.TURN3){
            turnDesiredAngle(turnSetpoint3);
            if(directionController.atSetpoint()){
                setState(States.STOP); 
            }
        } else if(states == States.DRIVE4){
            driveDesiredDistance(desiredTranslation);
            if(distanceController.atSetpoint()){
                preTDA(goalCoordinate4, goalCoordinate5); 
                setState(States.TURN2); 
            }
        } else if(states==States.TURN4){
            turnDesiredAngle(turnSetpoint4);
            if(directionController.atSetpoint()){
                setState(States.STOP); 
            }
        }else{
            drivetrainSubsystem.stopDrive();
        }
    }

    //predrivedesiredistance
    public Translation2d preDDD(Translation2d cCoordinate, Translation2d dCoordinate){
        double xDDistance = dCoordinate.getX() - cCoordinate.getX();
        double yDDistance = dCoordinate.getY() - cCoordinate.getY();
        hypotenuse = Math.hypot(xDDistance, yDDistance);
        distanceController.setSetpoint(hypotenuse);
        return new Translation2d (xDDistance, yDDistance);    
    }

    //d is desired coordinate
    //c is current coordinate
    @Override
    public void driveDesiredDistance(Translation2d dTranslation){      
        
        double speed = distanceController.calculate(Math.hypot(drivetrainSubsystem.m_pose.getX(), drivetrainSubsystem.m_pose.getY()), hypotenuse);
        double directionX = dTranslation.getX() / Math.sqrt(Math.pow(dTranslation.getX(),2) + Math.pow(dTranslation.getY(),2));
        double directionY = dTranslation.getY() / Math.sqrt(Math.pow(dTranslation.getX(),2) + Math.pow(dTranslation.getY(),2));
        
        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(speed * directionX, speed * directionY, 0, drivetrainSubsystem.getGyroscopeRotation()));  
    }

    //pre turn desired angle
    public void preTDA(Translation2d uno, Translation2d dos){
        directionController.reset();
        System.out.println("Preturning");
        //directionController.setSetpoint(setpoint);
        //drivetrainSubsystem.resetOdometry();
        autoCalculateAngle(uno, dos); 
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

    public double autoCalculateDistance(Translation2d initPose, Translation2d targetPose){
        double distancePose = Math.hypot(targetPose.getX() - initPose.getX(), targetPose.getY() - initPose.getY());
        return distancePose;

      }
      public double autoCalculateAngle(Translation2d initPose, Translation2d targetPose){
        double nextanglePose = (Math.acos(targetPose.getX() - initPose.getX()))/(Math.hypot(targetPose.getX() - initPose.getX(), targetPose.getY() - initPose.getY()));
        double angleTurn = 180 - drivetrainSubsystem.getGyroscopeRotation().getDegrees()  - (nextanglePose)*180/Math.PI;
        return angleTurn;
      }

}
