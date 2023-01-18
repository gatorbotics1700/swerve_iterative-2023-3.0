package frc.robot.autonomous;

import javax.naming.spi.DirStateFactory;
import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;

public class AutonomousBasePD extends AutonomousBase{
    public static double turnKP= 0.002;//deleted final to make them changable in shuffleboard
    public static double turnKI= 0.0;
    public static double turnKD= 0.0;
    public static double driveKP= 0.00004;
    public static double driveKI= 0.0;
    public static double driveKD= 0.0;
    
    public static double veloKP = 0.25;
    public static double veloKI = 0.3; 
    public static double veloKD = 0.35;

    private final double DEADBAND = 3;
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
    private PIDController velocityController = new PIDController(veloKP, veloKI, veloKD);
    
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
        directionController.setTolerance(DEADBAND); 
        distanceController.setTolerance(DEADBAND*Constants.TICKS_PER_INCH);
        states = States.FIRST;
        System.out.println("init!");
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
            System.out.println("we've reset to this pose: " + drivetrainSubsystem.m_pose);
            setState(States.DRIVE);
        }
        if (states == States.DRIVE){
            driveDesiredDistance(goalCoordinate1);
            System.out.println("inside drive state! pose: " + drivetrainSubsystem.m_pose.getX()/Constants.TICKS_PER_INCH + " " + drivetrainSubsystem.m_pose.getY()/Constants.TICKS_PER_INCH);
            if (distanceController.atSetpoint()){
                preTDA(turnSetpoint1);
                setState(States.TURN);
            }
        } else if(states==States.TURN){
            turnDesiredAngle(turnSetpoint1);
            if(directionController.atSetpoint()){
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

    //getters and setters for shuffleboard pid tab
    public double getTurnKP(){return turnKP;}
    public double getTurnKI(){return turnKI;}
    public double getTurnKD(){return turnKD;}

    public double getDriveKP(){return driveKP;}
    public double getDriveKI(){return driveKI;}
    public double getDriveKD(){return driveKD;}

    // public double getPitchKP(){return pitchKP;}//arbitrary placeholder #
    // public double getPitchKI(){return pitchKI;}
    // public double getPitchKD(){return pitchKD;}//^^

    public double getVeloKP(){return veloKP;}
    public double getVeloKI(){return veloKI;}
    public double getVeloKD(){return veloKD;}

    public void setTurnKP(double value){turnKP = value;}
    public void setTurnKI(double value){turnKI = value;}
    public void setTurnKD(double value){turnKD = value;}

    public void setDriveKP(double value){driveKP = value;}
    public void setDriveKI(double value){driveKI = value;}
    public void setDriveKD(double value){driveKD = value;}

    // public void setPitchKP(double value){pitchKP = value;}
    // public void setPitchKI(double value){pitchKI = value;}
    // public void setPitchKD(double value){pitchKD = value;}

    public void setVeloKP(double value){veloKP = value;}
    public void setVeloKI(double value){veloKI = value;}
    public void setVeloKD(double value){veloKD = value;}

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

    // public void pitchBalance(double pitchSetpoint){
    //     pitchController.setSetpoint(pitchSetpoint); 
    //     double error = pitchController.calculate(drivetrainSubsystem.m_pigeon.getPitch(), pitchSetpoint);
    //     System.out.println("error: " + error); 
    //     drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(pitchKP*error, 0 , 0, drivetrainSubsystem.getGyroscopeRotation()));
        
    //     if (Math.abs(drivetrainSubsystem.m_pigeon.getPitch()) - pitchSetpoint < 2.5){
    //         drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrainSubsystem.getGyroscopeRotation()));
    //         //velocityPD(0);
    //     }
    // }

    // public void velocityPD(double velocitySetpoint){
    //     velocityController.setSetpoint(velocitySetpoint);
    //     double veloDiff = velocityController.calculate(drivetrainSubsystem.getAverage(), velocitySetpoint);
    //     drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(veloKP*veloDiff, 0 , 0, drivetrainSubsystem.getGyroscopeRotation()));
    // }

}
