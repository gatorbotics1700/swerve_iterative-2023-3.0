package frc.robot.autonomous;

import javax.swing.SwingWorker.StateValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private static final double turnKP= 0.0001; //increased slight *** not tested
    private static final double turnKI= 0.0; 
    private static final double turnKD= 0.0;
    private static final double driveKP= 0.75; //Robot.kP.getDouble(0.00006);//0.00006;
    private static final double driveKI= 0.0; //Robot.kI.getDouble(0.0);//0.0;
    private static final double driveKD= 0.0; //Robot.kD.getDouble(0.0);//0.0;
    private static final double DRIVE_DEADBAND = 6*Constants.METERS_PER_INCH; //meters - previously 3 inches
    private static final double TURN_DEADBAND = 6.0; 

    private Pose2d startingCoordinate;
    private StateWithCoordinate[] stateSequence;
    private int i;
    private Boolean isFirst;
    private double startTime;
    private double startTime2;
    
    private DrivetrainSubsystem drivetrainSubsystem;
    private Mechanisms mechanisms;
    public AutoStates states;
    private AutonomousBaseEngage autoEngage; 

    //pids
    private PIDController turnController;
    private PIDController xController;
    private PIDController yController;

    public AutonomousBasePD(Pose2d startingCoordinate, StateWithCoordinate[] stateSequence){
        this.startingCoordinate = startingCoordinate;
        this.stateSequence =  stateSequence;
        init();
    }

    @Override
    public void init(){
        System.out.println("AUTONOMOUS INIT!\nINIT!\nINIT!");
        drivetrainSubsystem = Robot.m_drivetrainSubsystem;
        if(Robot.mechanismsEnabled){
            mechanisms = Robot.m_mechanisms;
        }
        drivetrainSubsystem.resetOdometry(startingCoordinate);
        turnController = new PIDController(turnKP, turnKI, turnKD); 
        xController = new PIDController(driveKP, driveKI, driveKD);
        yController = new PIDController(driveKP, driveKI, driveKD);
        autoEngage = new AutonomousBaseEngage(3);
        xController.reset();
        yController.reset();
        turnController.reset();
        turnController.enableContinuousInput(0, 360);
        i = 0;
        isFirst = true;
        startTime = 0.0;
        startTime2 = 0.0;
    }

    @Override
    public void periodic()
    {
        states = stateSequence[i].autoState;
        System.out.println("state: " + states);
        if (states == AutoStates.FIRST){
            turnController.setTolerance(TURN_DEADBAND); 
            xController.setTolerance(DRIVE_DEADBAND);
            yController.setTolerance(DRIVE_DEADBAND);
            xController.setSetpoint(0); //translation not pose component
            yController.setSetpoint(0);
            turnController.setSetpoint(0);
            i++;  
            System.out.println("moving on to " + stateSequence[i]);
            return;
        }
        //System.out.println("pose in auto: " + DrivetrainSubsystem.m_pose.getX()/Constants.METERS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getY()/Constants.METERS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getRotation().getDegrees());
        if (states == AutoStates.DRIVE){
            driveDesiredDistance(stateSequence[i].coordinate);
            if(Robot.mechanismsEnabled){
                mechanisms.setState(MechanismStates.HOLDING);
            }
            if(xController.atSetpoint() && yController.atSetpoint() && turnController.atSetpoint()){
                i++;  
                System.out.println("moving on to " + stateSequence[i]);
            }
        }else if( states == AutoStates.MIDNODE){
            if(Robot.mechanismsEnabled){
                System.out.println("mid node");
                mechanisms.setState(MechanismStates.MID_NODE);
                if(isFirst){
                    startTime2 = System.currentTimeMillis();
                }
                // If we never get to mid it might be because the timeout is too short
                if(mechanisms.isDoneMid()==true || System.currentTimeMillis() - startTime2 >= 2000){
                    if(isFirst){
                        startTime = System.currentTimeMillis();
                        isFirst = false;
                    }
                    mechanisms.pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeStates.RELEASING);
                    if (System.currentTimeMillis()-startTime>=1000){ //time to outtake before moving on
                        i++;
                        isFirst = true;
                    }
                } 
            }else{
                System.out.println("mid skipped bc mechanismsEnabled was false");
                i++;
            }
        }else if(states == AutoStates.LOWNODE){
            if(Robot.mechanismsEnabled){
                System.out.println("low node");
                mechanisms.setState(MechanismStates.LOW_NODE);
                if(isFirst){
                    startTime = System.currentTimeMillis();
                    isFirst = false;
                }
                // If we never get to low it might be because the timeout is too short
                if(mechanisms.isDoneLow()==true){
                    mechanisms.pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeStates.RELEASING);
                    if (System.currentTimeMillis()-startTime>=1000){
                        i++;
                        isFirst = true;
                    }
                }
            }else{
                System.out.println("low skipped bc mechanismsEnabled was false");
                i++;
            }
        } else if(states == AutoStates.ENGAGE){
            if(isFirst){
                isFirst = false;
            }
            autoEngage.periodic();   
        } else if(states == AutoStates.PICKUP){ // TODO: are left and right pickup supposed to be the same? if so, can we have just one state?
           if(Robot.mechanismsEnabled){     
            System.out.println("left pickup");
                if(isFirst){
                    startTime = System.currentTimeMillis();
                    isFirst = false;
                }
                // If we never get to shelf it might be because the timeout is too short
                if(mechanisms.isDoneShelf() == true || System.currentTimeMillis() - startTime >= 500){
                    i++;
                    isFirst = true;
                }
                // If we never get to shelf it might be because the timeout is too short
                if(mechanisms.isDoneShelf() == true || System.currentTimeMillis() - startTime >= 500){
                    i++;
                    isFirst = true;
                }
           }else{
                System.out.println("pickup skipped bc mechanismsEnabled was false");
                i++;
           }
        } else if(states == AutoStates.INTAKING){
            if(Robot.mechanismsEnabled){
                if(isFirst){
                    startTime = System.currentTimeMillis(); 
                    isFirst = false;
                }
                if(System.currentTimeMillis()-startTime>=500){
                    i++;
                    isFirst = true;
                }
                mechanisms.setState(MechanismStates.GROUNDPICKUP);
                //pneumaticIntakeSubsystem.setState(PneumaticIntakeStates.ACTUATING);TODO uncomment
                if(states == AutoStates.FASTDRIVE){
                    if(Math.abs(stateSequence[i].coordinate.getX() - drivetrainSubsystem.getMPoseX())>DRIVE_DEADBAND){
                        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(1,0,0, drivetrainSubsystem.getPoseRotation()));
                        mechanisms.setState(MechanismStates.HOLDING);
                    }else{
                        drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, drivetrainSubsystem.getPoseRotation()));
                        i++;
                    }
                } else {
                    drivetrainSubsystem.stopDrive();
                } 
                drivetrainSubsystem.drive(); 
        }else{
                System.out.println("intaking skipped bc mechanismsEnabled was false");
                i++;
            }
        }
    }

    /** 
    @param dPose is desired pose
    */
    @Override
    public void driveDesiredDistance(Pose2d dPose){      
        //System.out.println("xcontroller setpoint: " + xController.getSetpoint());
        //System.out.println("cur pose: " + DrivetrainSubsystem.m_pose);
        //System.out.println("desired pose: " + dPose);
        double speedX = xController.calculate(drivetrainSubsystem.getMPoseX(), dPose.getX());
        double speedY = yController.calculate(drivetrainSubsystem.getMPoseY(), dPose.getY());
        System.out.println("m_pose deg: " + drivetrainSubsystem.getMPoseDegrees() % 360);
        System.out.println("d_pose deg: " + dPose.getRotation().getDegrees() % 360);
        double speedRotat = turnController.calculate(drivetrainSubsystem.getMPoseDegrees(), dPose.getRotation().getDegrees());
        //System.out.println("DDDing");
        //System.out.println("speed rotate: " + speedRotat);
        System.out.println("sp");
        
        if(xAtSetpoint()){ //TODO: remove this once the built in setpoints work in vision
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
        double errorX = (dPose.getX() - drivetrainSubsystem.getMPoseX());
        double errorY = (dPose.getY() - drivetrainSubsystem.getMPoseY());
        double errorRotat = turnController.getPositionError();
        System.out.println("Rotation error: " + errorRotat + " deadband " + turnController.getPositionTolerance());
        System.out.println("Speed X: " + speedX + " Speed Y: " + speedY + " Speed R: " + speedRotat);
        //System.out.println("error:" + errorX + ", " + errorY + ", " + errorRotat);
        //System.out.println("Desired Position: " + dPose.getX() + ", " + dPose.getY());
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

    public void setState(StateWithCoordinate.AutoStates newAutoState){
        states = newAutoState;
    }
}