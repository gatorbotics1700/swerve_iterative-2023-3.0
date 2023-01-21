package frc.robot.autonomous;

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
    public static final double driveKP= 0.00006;
    public static final double driveKI= 0.0;
    public static final double driveKD= 0.0;
    private final double DRIVE_DEADBAND = 3;
    private final double TURN_DEADBAND = 6;
    private double hypotenuse;

    private Translation2d goalCoordinate1;
    private Translation2d goalCoordinate2; 
    private Translation2d goalCoordinate3; 
    private Translation2d goalCoordinate4; 
    private Translation2d goalCoordinate5;
    private Translation2d goalCoordinate6;

    public double desiredTurn;
    private Translation2d desiredTranslation;

    DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

    //pids
    private PIDController directionController = new PIDController(turnKP, turnKI, turnKD);
    private PIDController distanceController = new PIDController(driveKP, driveKI, driveKD);
    
    public AutonomousBasePD(Translation2d goalCoordinate1, Translation2d goalCoordinate2, Translation2d goalCoordinate3, Translation2d goalCoordinate4, Translation2d goalCoordinate5, Translation2d goalCoordinate6){
        this.goalCoordinate1 = goalCoordinate1;
        this.goalCoordinate2 = goalCoordinate2;
        this.goalCoordinate3 = goalCoordinate3;
        this.goalCoordinate4 = goalCoordinate4;
        this.goalCoordinate5 = goalCoordinate5;
        this.goalCoordinate6 = goalCoordinate6;
    }

    @Override
    public void init(){
        drivetrainSubsystem.resetOdometry();
        // directionController.reset();
        // distanceController.reset();
        directionController.setTolerance(TURN_DEADBAND); 
        distanceController.setTolerance(DRIVE_DEADBAND*Constants.TICKS_PER_INCH);
        states = States.FIRST;
        System.out.println("INIT!\nINIT!\nINIT!");

    }

    public static enum States{
        FIRST,
        DRIVE,
        DRIVE2,
        DRIVE3,
        DRIVE4,
        STOP;
    
    }

    private static States states = States.FIRST; //change later

    public void setState(States newState){
        states = newState;
    }

    @Override
    public void periodic()
    {
        
        //System.out.println("state: "+states);
        if (states == States.FIRST){
            desiredTranslation = preDDD(new Translation2d(), goalCoordinate1); 
            System.out.println("we've reset to this pose: " + DrivetrainSubsystem.m_pose);
            setState(States.DRIVE);
        }
        if (states == States.DRIVE){
            driveDesiredDistance(desiredTranslation);
            System.out.println("inside drive state! pose: " + drivetrainSubsystem.m_pose.getX()/Constants.TICKS_PER_INCH + " " + drivetrainSubsystem.m_pose.getY()/Constants.TICKS_PER_INCH);
            if (distanceController.atSetpoint()){
                preTDA(goalCoordinate1, goalCoordinate2);
                setState(States.DRIVE2);
            }
        } else if(states == States.DRIVE2){
            driveDesiredDistance(desiredTranslation);
            if(distanceController.atSetpoint()){
                preTDA(goalCoordinate2, goalCoordinate3); 
                setState(States.DRIVE3);  
            }
        } else if(states == States.DRIVE3){
            driveDesiredDistance(desiredTranslation);
            if(distanceController.atSetpoint()){
                preTDA(goalCoordinate3, goalCoordinate4); 
                setState(States.DRIVE4); 
            }
        } else if(states == States.DRIVE4){
            driveDesiredDistance(desiredTranslation);
            if(distanceController.atSetpoint()){
                preTDA(goalCoordinate4, goalCoordinate5);
                setState(States.STOP); 
            }
            //one more drive, one more turn needed (final turn should bring you toooo facing wall to deposit!)
        }else{
            drivetrainSubsystem.stopDrive();
        }
    }

    //predrivedesiredistance
    public Translation2d preDDD(Translation2d cCoordinate, Translation2d dCoordinate){
        double xDDistance = Constants.TICKS_PER_INCH*(dCoordinate.getX() - cCoordinate.getX());
        double yDDistance = Constants.TICKS_PER_INCH*(dCoordinate.getY() - cCoordinate.getY());
        hypotenuse = Math.hypot(xDDistance, yDDistance);
        distanceController.setSetpoint(hypotenuse); 
        System.out.println("preDDDing: " + xDDistance + ", " + yDDistance);    
        return new Translation2d (xDDistance, yDDistance);
    }

    /** 
    @param dTranslation is desired 
    */
    @Override
    public void driveDesiredDistance(Translation2d dTranslation){      
<<<<<<< HEAD
        System.out.println("where we are rn: " + drivetrainSubsystem.m_pose.getX() + " and " + drivetrainSubsystem.m_pose.getY());
        double speed = distanceController.calculate(Math.hypot(drivetrainSubsystem.m_pose.getX(), drivetrainSubsystem.m_pose.getY()), hypotenuse);
=======
        double speed = distanceController.calculate(Math.hypot(DrivetrainSubsystem.m_pose.getX(), DrivetrainSubsystem.m_pose.getY()), hypotenuse);
>>>>>>> 6ba44927ffd48b007c48714463d94fc25af13901
        double directionX = dTranslation.getX() / Math.sqrt(Math.pow(dTranslation.getX(),2) + Math.pow(dTranslation.getY(),2));
        double directionY = dTranslation.getY() / Math.sqrt(Math.pow(dTranslation.getX(),2) + Math.pow(dTranslation.getY(),2));
        System.out.println("DDDing");    
        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(speed * directionX, speed * directionY, 0, drivetrainSubsystem.getGyroscopeRotation()));  

        System.out.println("current speed: " + speed);
    }

    /**
    pre turn desired angle 
    @param uno is current coordinate
    @param dos is past coordinate 
    */
    public void preTDA(Translation2d uno, Translation2d dos){
        //directionController.reset();
        System.out.println("Preturning");
        desiredTurn = autoCalculateAngle(uno, dos); 
    }

    @Override
    public void turnDesiredAngle(double desiredTurn){
        System.out.println("desired turn: " + desiredTurn);
        directionController.enableContinuousInput(0, 360); //so it goes shortest angle to get to correct
        double pidturnval = directionController.calculate(drivetrainSubsystem.getGyroscopeRotation().getDegrees(), desiredTurn);
        System.out.println("pid val: " + pidturnval);
        drivetrainSubsystem.setSpeed(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
            Math.signum(pidturnval)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE-0.1, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(pidturnval))), 
            drivetrainSubsystem.getGyroscopeRotation())
        );
        System.out.println("error: " + directionController.getPositionError());
    }

      public double autoCalculateAngle(Translation2d initPose, Translation2d targetPose){ 
        double xleg = Math.abs(targetPose.getX() - initPose.getX());
        double hypo = Math.hypot(targetPose.getX() - initPose.getX(), targetPose.getY() - initPose.getY());
        double theta = Math.toDegrees(Math.acos(xleg/hypo));
        System.out.println("theta: " + theta + " xleg: " + xleg + " hypo: " + hypo);
        //theta = angle calculated according horizontal distance between initpose and targetpose, as well as hypotenuse
        if(targetPose.getX() >= initPose.getX() && targetPose.getY() >= initPose.getY()){ //if initpose is considered (0,0), targetpose is in quadrant I
            return theta - drivetrainSubsystem.getGyroscopeRotation().getDegrees();
        } else if (targetPose.getX() <= initPose.getX() && targetPose.getY() >= initPose.getY()){ //if targetpose is in quadrant II
            return 180 - theta - drivetrainSubsystem.getGyroscopeRotation().getDegrees();
        } else if (targetPose.getX() <= initPose.getX() && targetPose.getY() <= initPose.getY()){ //if targetpose is in quadrant III
            return 180 + theta - drivetrainSubsystem.getGyroscopeRotation().getDegrees();
        } else if (targetPose.getX() >= initPose.getX() && targetPose.getY() <= initPose.getY()){ //if targetpose is in quadrant IV
            return 360 - theta - drivetrainSubsystem.getGyroscopeRotation().getDegrees();
        }
        System.out.println("Something wrong w/ angle calculation :(");
        return 0.0;
      }

}
