package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class AutonomousBasePD extends AutonomousBase{
    public static final double turnKP= 0.0001; //increased slight *** not tested
    public static final double turnKI= 0.0;
    public static final double turnKD= 0.0;
    public static final double driveKP= 0.02;//Robot.kP.getDouble(0.00006);//0.00006;
    public static final double driveKI= 0.0; //Robot.kI.getDouble(0.0);//0.0;
    public static final double driveKD= 0.0; //Robot.kD.getDouble(0.0);//0.0;
    private final double DRIVE_DEADBAND = 0.0762; //meters - previously 3 inches
    private final double TURN_DEADBAND = 6; 

    
    private Pose2d startingCoordinate;
    private Pose2d goalCoordinate1;
    private Pose2d goalCoordinate2; 
    private Pose2d goalCoordinate3; 
    private Pose2d goalCoordinate4; 
    private Pose2d goalCoordinate5;
    private Pose2d goalCoordinate6;
    private double goalAngle1;
    

    public double desiredTurn;
    
    DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

    //pids
    private PIDController directionController = new PIDController(turnKP, turnKI, turnKD);
    private PIDController turnController = new PIDController(turnKP, turnKI, turnKD); 
    private PIDController xController = new PIDController(driveKP, driveKI, driveKD);
    private PIDController yController = new PIDController(driveKP, driveKI, driveKD);


    
    public AutonomousBasePD(Pose2d startingCoordinate, Pose2d goalCoordinate1, Pose2d goalCoordinate2, Pose2d goalCoordinate3, Pose2d goalCoordinate4, Pose2d goalCoordinate5, Pose2d goalCoordinate6, double goalAngle1){
        this.startingCoordinate = startingCoordinate;
        this.goalCoordinate1 = goalCoordinate1;
        this.goalCoordinate2 = goalCoordinate2;
        this.goalCoordinate3 = goalCoordinate3;
        this.goalCoordinate4 = goalCoordinate4;
        this.goalCoordinate5 = goalCoordinate5;
        this.goalCoordinate6 = goalCoordinate6;
        this.goalAngle1 = goalAngle1;
    }

    public AutonomousBasePD(){
        startingCoordinate = new Pose2d(0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation());
        goalCoordinate1 = new Pose2d(0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation());
        goalCoordinate2 = new Pose2d(0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation());
        goalCoordinate3 = new Pose2d(0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation());
        goalCoordinate4 = new Pose2d(0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation());
        goalCoordinate5 = new Pose2d(0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation());
        goalCoordinate6 = new Pose2d(0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation());
        //goalAngle1 = drivetrainSubsystem.getGyroscopeRotation();
    }
    @Override
    public void init(){
        drivetrainSubsystem.resetOdometry(startingCoordinate);
        directionController.reset();
        xController.reset();
        yController.reset();
        turnController.reset();
        directionController.setTolerance(TURN_DEADBAND);
        turnController.setTolerance(TURN_DEADBAND); 
        xController.setTolerance(DRIVE_DEADBAND);
        yController.setTolerance(DRIVE_DEADBAND);

        states = States.FIRST;
        System.out.println("INIT!\nINIT!\nINIT!");

    }

    public static enum States{
        FIRST,
        DRIVE,
       // TURN1,
        DRIVE2,
        DRIVE3,
        DRIVE4,
        DRIVE5,
        DRIVE6,
        STOP;
    
    }

    private static States states = States.FIRST; 

    public void setState(States newState){
        states = newState;
    }

    @Override
    public void periodic()
    {
        
       // System.out.println("state: " + states);
        if (states == States.FIRST){
            //System.out.println("we've reset to this pose: " + DrivetrainSubsystem.m_pose);
            setState(States.DRIVE);
            xController.setSetpoint(goalCoordinate1.getX()); 
            yController.setSetpoint(goalCoordinate1.getY());
            turnController.setSetpoint(goalCoordinate1.getRotation().getDegrees());
        } else {
            drivetrainSubsystem.drive();
            if (states == States.DRIVE){
                driveDesiredDistance(goalCoordinate1);
            //     //System.out.println("inside drive state! pose: " + DrivetrainSubsystem.m_pose.getX()/Constants.TICKS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getY()/Constants.TICKS_PER_INCH);
            //     if (xController.atSetpoint() && yController.atSetpoint() && turnController.atSetpoint()){
            //        setState(States.TURN1); 
            //        System.out.println("Position: " + DrivetrainSubsystem.m_pose.getX()/Constants.TICKS_PER_INCH + ", " + DrivetrainSubsystem.m_pose.getY()/Constants.TICKS_PER_INCH);
            //     }
            // } else if(states == States.TURN1){
            //     turnDesiredAngle(goalAngle1);
                if(xController.atSetpoint() && yController.atSetpoint() /*&& turnController.atSetpoint()*/){
                    setState(States.DRIVE2);  
                  //  System.out.println("Position: " + DrivetrainSubsystem.m_pose.getX() + ", " + DrivetrainSubsystem.m_pose.getY());
                }
            } else if(states == States.DRIVE2){
                driveDesiredDistance(goalCoordinate2);
                if(xController.atSetpoint() && yController.atSetpoint()){
                    setState(States.DRIVE3); 
                //    System.out.println("Position: " + DrivetrainSubsystem.m_pose.getX() + ", " + DrivetrainSubsystem.m_pose.getY());
                }
            } else if(states == States.DRIVE3){
                driveDesiredDistance(goalCoordinate3);
                if(xController.atSetpoint() && yController.atSetpoint()){
                    setState(States.DRIVE4); 
                 //   System.out.println("Position: " + DrivetrainSubsystem.m_pose.getX() + ", " + DrivetrainSubsystem.m_pose.getY());
                }
            } else if(states == States.DRIVE4){
                driveDesiredDistance(goalCoordinate4);
                if(xController.atSetpoint() && yController.atSetpoint()){
                    setState(States.DRIVE5); 
                  //  System.out.println("Position: " + DrivetrainSubsystem.m_pose.getX() + ", " + DrivetrainSubsystem.m_pose.getY());
                }
            }else if(states==States.DRIVE5){
                driveDesiredDistance(goalCoordinate5);
                if(xController.atSetpoint() && yController.atSetpoint()){
                    setState(States.DRIVE6);
                  //  System.out.println("Position: " + DrivetrainSubsystem.m_pose.getX() + ", " + DrivetrainSubsystem.m_pose.getY());
                }
            }else if(states==States.DRIVE6){
                driveDesiredDistance(goalCoordinate6);
                if(xController.atSetpoint() && yController.atSetpoint()){
                    setState(States.STOP);
                   // System.out.println("Position: " + DrivetrainSubsystem.m_pose.getX() + ", " + DrivetrainSubsystem.m_pose.getY());
                }    
            }else{
                drivetrainSubsystem.stopDrive();
            }
        }
    }

    //predrivedesiredistance
    public void preDDD(Pose2d cCoordinate, Pose2d dCoordinate){
        //Rotation2d zDDistance = new Rotation2d(Constants.TICKS_PER_INCH*(dCoordinate.getRotation().getDegrees() - cCoordinate.getRotation().getDegrees()));
        xController.setSetpoint(dCoordinate.getX()); 
        yController.setSetpoint(dCoordinate.getY());
        System.out.println("preDDDing!!!");    
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
            System.out.println("At setpoint");
        } else {
            //System.out.println("Position error: " + turnController.getPositionError());
            speedRotat = Math.signum(speedRotat)*Math.max(Constants.STEER_MOTOR_MIN_VOLTAGE, Math.min(Constants.STEER_MOTOR_MAX_VOLTAGE, Math.abs(speedRotat)));
          //  System.out.println("Speed rotat after: " + speedRotat);
        }

        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRotat, drivetrainSubsystem.getGyroscopeRotation()));  
        double errorX = (dPose.getX() - DrivetrainSubsystem.m_pose.getX());
        double errorY = (dPose.getY() - DrivetrainSubsystem.m_pose.getY());
        double errorRotat = (dPose.getRotation().getDegrees() - DrivetrainSubsystem.m_pose.getRotation().getDegrees());
        // System.out.println("Speed X: " + speedX + " Speed Y: " + speedY + " Speed Rotat: " + speedRotat);
        // System.out.println("error:" + errorX + ", " + errorY + ", " + errorRotat);
        //System.out.println("Desired Position: " + dPose.getX() + ", " + dPose.getY());
    }

    /**
    pre turn desired angle 
    @param uno is current coordinate
    @param dos is past coordinate 
    */
    public void preTDA(Pose2d uno, Pose2d dos){
        System.out.println("Preturning");
        desiredTurn = autoCalculateAngle(uno, dos); 
    }

    @Override
    public void turnDesiredAngle(double desiredTurn){
      //  System.out.println("desired turn: " + desiredTurn);
        directionController.enableContinuousInput(0, 360); //so it goes shortest angle to get to correct
        double pidturnval = directionController.calculate(drivetrainSubsystem.getGyroscopeRotation().getDegrees(), desiredTurn);
       // System.out.println("pid val: " + pidturnval);
        drivetrainSubsystem.setSpeed(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
            Math.signum(pidturnval)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE-0.1, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(pidturnval))), 
            drivetrainSubsystem.getGyroscopeRotation())
        );
       // System.out.println("error: " + directionController.getPositionError());
    }

      public double autoCalculateAngle(Pose2d initPose, Pose2d targetPose){ 
        double xleg = Math.abs(targetPose.getX() - initPose.getX());
        double hypo = Math.hypot(targetPose.getX() - initPose.getX(), targetPose.getY() - initPose.getY());
        double theta = Math.toDegrees(Math.acos(xleg/hypo));
       // System.out.println("theta: " + theta + " xleg: " + xleg + " hypo: " + hypo);
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
