package frc.robot.autonomous;

import javax.swing.SwingWorker.StateValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.subsystems.ArmPneumaticPivot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.autonomous.StateWithCoordinate;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem;
//import frc.robot.subsystems.ArmPneumaticPivot.PneumaticPivotStates;
import frc.robot.subsystems.Mechanisms.MechanismStates;
//import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates; TODO uncomment
import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.Vision.LimeLightSubsystem;
import frc.robot.subsystems.Vision.AprilTagSubsystem;

public class AutonomousBasePD extends AutonomousBase{
    private static final double turnKP= 0.0001; //increased slight *** not tested
    private static final double turnKI= 0.0; 
    private static final double turnKD= 0.0;
    private static final double driveKP= 0.75; //Robot.kP.getDouble(0.00006);//0.00006;
    private static final double driveKI= 0.0; //Robot.kI.getDouble(0.0);//0.0;
    private static final double driveKD= 0.0; //Robot.kD.getDouble(0.0);//0.0;
    private static final double DRIVE_DEADBAND = 2*Constants.METERS_PER_INCH; //meters - previously 3 inches
    private static final double TURN_DEADBAND = 6.0; 

    private Pose2d startingCoordinate;
    private StateWithCoordinate[] stateSequence;
    private int i;
    private Boolean isFirst;
    private double startTime;
    
    private ArmPneumaticPivot armPneumaticPivot;
    private DrivetrainSubsystem drivetrainSubsystem;
    private Mechanisms mechanisms;
    private PneumaticIntakeSubsystem pneumaticIntakeSubsystem;
    private LimeLightSubsystem limeLightSubsystem;
    public AutoStates states;

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
        armPneumaticPivot = Mechanisms.armPneumaticPivot;
        drivetrainSubsystem = Robot.m_drivetrainSubsystem;
        mechanisms = Robot.m_mechanisms;
        //pneumaticIntakeSubsystem = Robot.m_pneumaticIntakeSubsystem;
        limeLightSubsystem = AprilTagSubsystem.limeLightSubsystem;
        drivetrainSubsystem.resetOdometry(startingCoordinate);
        turnController = new PIDController(turnKP, turnKI, turnKD); 
        xController = new PIDController(driveKP, driveKI, driveKD);
        yController = new PIDController(driveKP, driveKI, driveKD);
        xController.reset();
        yController.reset();
        turnController.reset();
        turnController.enableContinuousInput(0, 360);
        i = 0;
        isFirst = true;
        startTime = 0.0;
    }

    @Override
    public void periodic()
    {
        states = stateSequence[i].state;
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
        } else if (states == AutoStates.FIRSTHIGHNODE){ //WE HAVE THIS BECAUSE OF APRILTAGS
            doHighNode();
            return;
        }
        
        //System.out.println("pose in auto: " + DrivetrainSubsystem.m_pose.getX()/Constants.METERS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getY()/Constants.METERS_PER_INCH + " " + DrivetrainSubsystem.m_pose.getRotation().getDegrees());
        if (states == AutoStates.DRIVE){
            driveDesiredDistance(stateSequence[i].coordinate);
            if(xController.atSetpoint() && yController.atSetpoint() && turnController.atSetpoint()){
                i++;  
                System.out.println("moving on to " + stateSequence[i]);
            }
        }else if(states == AutoStates.HIGHNODE){ 
            doHighNode();
        }else if(states == AutoStates.MIDNODE){
            System.out.println("mid node");
            mechanisms.setState(MechanismStates.MID_NODE);
            if(isFirst){
                startTime = System.currentTimeMillis();
                isFirst = false;
            }
            // If we never get to mid it might be because the timeout is too short
            if(mechanisms.isDoneMid()==true || System.currentTimeMillis()-startTime>=500){
                    i++;
                    isFirst = true;
            }
        }else if(states == AutoStates.LOWNODE){
            System.out.println("low node");
            mechanisms.setState(MechanismStates.LOW_NODE);
            if(isFirst){
                startTime = System.currentTimeMillis();
                isFirst = false;
            }
            // If we never get to low it might be because the timeout is too short
            if(mechanisms.isDoneLow()==true || System.currentTimeMillis()-startTime>=500){
                i++;
                isFirst = true;
            }
        }else if(states == AutoStates.PICKUP){ // TODO: are left and right pickup supposed to be the same? if so, can we have just one state?
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
        }else if(states == AutoStates.BALANCING){
            //armPneumaticPivot.setState(PneumaticPivotStates.ACTUATING);
            Robot.m_drivetrainSubsystem.pitchBalance(0.0);
        }else if(states == AutoStates.INTAKING){
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
        }else{
            drivetrainSubsystem.stopDrive();
        } 
        drivetrainSubsystem.drive();  
    }

    private void doHighNode() {
        mechanisms.setState(MechanismStates.HIGH_NODE);
        System.out.println("high node");
        if(isFirst){
            startTime = System.currentTimeMillis();
            isFirst = false;
        }
        // Start a timer in case we never finish getting to high
        if(mechanisms.isDoneHigh()==true || System.currentTimeMillis()-startTime>=500){
            i++;
            isFirst = true;
            System.out.println("moving on to " + stateSequence[i]);
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

    // TODO: look through this method when we're ready to use vision
    public void driveDesiredDistanceVision(Pose2d dPose){ 
        System.out.println("Limelight tv auto: " + limeLightSubsystem.getTv());

        turnController.enableContinuousInput(0, 360);
        //System.out.println("xcontroller setpoint: " + xController.getSetpoint());
        //System.out.println("cur pose: " + DrivetrainSubsystem.m_pose);
        //System.out.println("desired pose: " + dPose);
        double speedX = xController.calculate(drivetrainSubsystem.getMPoseX(), dPose.getX());
        double speedY = yController.calculate(drivetrainSubsystem.getMPoseY(), dPose.getY());
        //System.out.println("m_pose deg: " + DrivetrainSubsystem.m_pose.getRotation().getDegrees() % 360);
        //System.out.println("d_pose deg: " + dPose.getRotation().getDegrees() % 360);
        double speedRotat = turnController.calculate(drivetrainSubsystem.getMPoseDegrees(), dPose.getRotation().getDegrees());
        //System.out.println("DDDing");
        //System.out.println("speed rotate: " + speedRotat);
        
        if(xAtSetpoint() || limeLightSubsystem.getTv() == 0.0){
            speedX = 0; 
            //System.out.println("In x deadband.\nX controller error: " + xController.getPositionError() + " in meters.");
        } else {
            speedX = Math.signum(speedX)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(speedX)));  
        }
    
        if(yAtSetpoint() || limeLightSubsystem.getTv() == 0.0){
            speedY = 0; 
            //System.out.println("In y deadband.");
        } else {
            speedY = Math.signum(speedY)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(speedY)));
        }
    
        if(turnAtSetpoint() || limeLightSubsystem.getTv() == 0.0){
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

}