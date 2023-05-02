package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.OI;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Mechanisms;
//NOTES ON MEASUREMENTS
//22.5inches on inside of metal frame that chain moves in
//10 in on moving mechanism thing
import frc.robot.subsystems.Mechanisms.MechanismStates;

public class ElevatorSubsystem {
    private static final double _kP = 0.2;//0.15;
    private static final double _kI = 0.0;
    private static final double _kD = 0.0;
    private static final int _kIzone = 0;
    private static final double _kPeakOutput = 1.0;
    private static final double MID_HEIGHT_INCHES = 30;
    private static final double LOW_HEIGHT_INCHES = 0;
    private static final double SHELF_HEIGHT_INCHES = 30; 
    private static final double MAX_HEIGHT_INCHES = 31.5;

    private static final double ELEVATOR_SPROCKET_DIAMETER = 1.28;
    private static final double ELEVATOR_GEAR_RATIO = 25.0;
    private static final double ELEVATOR_TICKS_PER_INCH = Constants.TICKS_PER_REV*ELEVATOR_GEAR_RATIO/ELEVATOR_SPROCKET_DIAMETER/Math.PI;

    public TalonFX elevatorMotor;
    private ElevatorStates elevatorState;
    private Mechanisms mechanisms;
    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;
    public LimitSwitchStates limitSwitchStates;
    
    private Gains elevatorGains = new Gains(_kP, _kI, _kD, _kIzone, _kPeakOutput);
    private static final double DEADBAND = 5000; //15000;

    public static enum ElevatorStates{
        ZERO, LOW_ELEVATOR_HEIGHT, MID_ELEVATOR_HEIGHT, SHELF_ELEVATOR_HEIGHT, MANUAL, STOPPED; 
    }
    
    private enum LimitSwitchStates{
        TOOHIGH, TOOLOW, HEIGHTOKAY;
    }

    public ElevatorSubsystem(){
        elevatorMotor = new TalonFX(Constants.ELEVATOR_CAN_ID);
        topLimitSwitch = new DigitalInput(0);
        bottomLimitSwitch = new DigitalInput(1);
        init();
    }

    public void init(){
        System.out.println("elevator init!!!!");
        elevatorMotor.setInverted(true); // looking from the front of the robot, clockwise is false (:
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        setState(ElevatorStates.LOW_ELEVATOR_HEIGHT);
        //configuring deadband
        elevatorMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //TODO: figure out what this line does
		/* Config Position Closed Loop gains in slot0, typically kF stays zero. */
		elevatorMotor.config_kP(Constants.kPIDLoopIdx, elevatorGains.kP, Constants.kTimeoutMs);
		elevatorMotor.config_kI(Constants.kPIDLoopIdx, elevatorGains.kI, Constants.kTimeoutMs);
		elevatorMotor.config_kD(Constants.kPIDLoopIdx, elevatorGains.kD, Constants.kTimeoutMs);
        limitSwitchStates = LimitSwitchStates.HEIGHTOKAY;
        mechanisms = Robot.m_mechanisms;
    }

    public void periodic(){
        System.out.println("Elevator State: " + elevatorState);
        System.out.println("Limit Switch State " + limitSwitchStates);
        elevatorLimitPeriodic();
        elevatorPositionPeriodic();
    }
    
    public void elevatorLimitPeriodic(){
        if(topLimitSwitch.get()){
            limitSwitchStates = LimitSwitchStates.TOOHIGH;
        }else if(bottomLimitSwitch.get()){
            limitSwitchStates = LimitSwitchStates.TOOLOW;
        }else{
            limitSwitchStates = LimitSwitchStates.HEIGHTOKAY;
        }
    }

    public void elevatorPositionPeriodic(){
        if (elevatorState == ElevatorStates.ZERO){ //emergency stop
            setElevator(0);
        } else if (elevatorState == ElevatorStates.LOW_ELEVATOR_HEIGHT){
            double desiredTicks = determineRightTicks(LOW_HEIGHT_INCHES);
            setElevator(desiredTicks);
        } else if (elevatorState == ElevatorStates.SHELF_ELEVATOR_HEIGHT) {
            double desiredTicks = determineRightTicks(SHELF_HEIGHT_INCHES);
            setElevator(desiredTicks);
        } else if (elevatorState == ElevatorStates.MID_ELEVATOR_HEIGHT){
            double desiredTicks = determineRightTicks(MID_HEIGHT_INCHES);
            setElevator(desiredTicks);
        }else if (elevatorState == ElevatorStates.MANUAL){
            manualElevator();
        } else { //emergency stop again for safety
            elevatorMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    private double determineRightTicks(double desiredInches){
        return desiredInches * ELEVATOR_TICKS_PER_INCH; 
    }

    private void setElevator(double desiredTicks){
        if(Math.abs(desiredTicks - elevatorMotor.getSelectedSensorPosition()) > DEADBAND){
            if(limitSwitchStates == LimitSwitchStates.TOOHIGH){
                if(desiredTicks >= elevatorMotor.getSelectedSensorPosition()){
                    mechanisms.setState(MechanismStates.STOP);
                    System.out.println("Elevator state SHOULD BE STOPPED: " + elevatorState);
                    System.out.println("should be setting elevator state to STOP");
                }else{
                    elevatorMotor.set(ControlMode.Position, desiredTicks);
                }
            }else if(limitSwitchStates == LimitSwitchStates.TOOLOW){
                if(desiredTicks <= elevatorMotor.getSelectedSensorPosition()){
                    mechanisms.setState(MechanismStates.STOP);
                }else{
                    elevatorMotor.set(ControlMode.Position, desiredTicks);
                }
            }else{
                elevatorMotor.set(ControlMode.Position, desiredTicks);
            }
        }else{
            elevatorState = ElevatorStates.STOPPED;
        }
    }

    public void setState(ElevatorStates newElevatorState){
        elevatorState = newElevatorState;
    }

    private void manualElevator(){
        if(elevatorMotor.getSelectedSensorPosition() <= MAX_HEIGHT_INCHES * ELEVATOR_TICKS_PER_INCH /*&&
           (elevatorMotor.getSelectedSensorPosition() >= 0*/){
            if(OI.getRightAxis() < -0.2 && limitSwitchStates == LimitSwitchStates.HEIGHTOKAY){
                elevatorMotor.set(ControlMode.PercentOutput, 0.2);
            }else if(OI.getRightAxis() > 0.2 && limitSwitchStates != LimitSwitchStates.TOOLOW) {
                elevatorMotor.set(ControlMode.PercentOutput, -0.2);
            }else {
                elevatorMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            elevatorMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public boolean isAtMid(){
        return Math.abs(elevatorMotor.getSelectedSensorPosition()-MID_HEIGHT_INCHES*ELEVATOR_TICKS_PER_INCH)<DEADBAND;
    }

    public boolean isAtLow(){
        return Math.abs(elevatorMotor.getSelectedSensorPosition()-LOW_HEIGHT_INCHES*ELEVATOR_TICKS_PER_INCH)<DEADBAND;
    }

    public boolean isAtShelf(){
        return Math.abs(elevatorMotor.getSelectedSensorPosition()-SHELF_HEIGHT_INCHES*ELEVATOR_TICKS_PER_INCH) < DEADBAND;
    }

    public boolean isAtZero(){
        return elevatorMotor.getSelectedSensorPosition() < DEADBAND;
    }

    public double getSelectedSensorPosition(){
        return elevatorMotor.getSelectedSensorPosition();
    }
}
