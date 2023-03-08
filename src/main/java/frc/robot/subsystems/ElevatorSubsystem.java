package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import frc.robot.Gains;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;

//NOTES ON MEASUREMENTS
//22.5inches on inside of metal frame that chain moves in
//10 in on moving mechanism thing

public class ElevatorSubsystem {

    public double _kP = 0.15;
    public double _kI = 0.0;
    public double _kD = 0.0;
    public int _kIzone = 0;
    public double _kPeakOutput = 1.0;

    public TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_CAN_ID);
    public static ElevatorStates elevatorState = ElevatorStates.LOW_ELEVATOR_HEIGHT;

    // DigitalInput top_limit_switch = new DigitalInput(Constants.topLimitSwitchPort);
    // DigitalInput bottom_limit_switch = new DigitalInput(Constants.bottomLimitSwitchPort);
    
    public Gains elevatorGains = new Gains(_kP, _kI, _kD, _kIzone, _kPeakOutput);
    public double desiredInches;
    public double deadband = 15000;

    public static enum ElevatorStates{
        ZERO, 
        LOW_ELEVATOR_HEIGHT,
        SHELF_ELEVATOR_HEIGHT,
        MID_ELEVATOR_HEIGHT,
        HIGH_ELEVATOR_HEIGHT;
    }

    public void init(){
        System.out.println("elevator init!!!!");
        elevatorMotor.setInverted(true); // looking from the front of the robot, clockwise is false (:
        elevatorMotor.setNeutralMode(NeutralMode.Brake);

        //configuring deadband
        elevatorMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		/* Config Position Closed Loop gains in slot0, typically kF stays zero. */
		elevatorMotor.config_kP(Constants.kPIDLoopIdx, elevatorGains.kP, Constants.kTimeoutMs);
		elevatorMotor.config_kI(Constants.kPIDLoopIdx, elevatorGains.kI, Constants.kTimeoutMs);
		elevatorMotor.config_kD(Constants.kPIDLoopIdx, elevatorGains.kD, Constants.kTimeoutMs);
    }

    public void periodic(){ //still need to scale down the values or figure out why it is overshooting
        System.out.println("current elevator motor position:" + elevatorMotor.getSelectedSensorPosition());
        if (elevatorState == ElevatorStates.ZERO){ //emergency stop
            elevatorMotor.set(ControlMode.Position, 0);
            System.out.println("desired ticks: 0");
            System.out.println("error: " + (0 - elevatorMotor.getSelectedSensorPosition()));
            elevatorDeadband(0);
        } else if (elevatorState == ElevatorStates.LOW_ELEVATOR_HEIGHT){
            desiredInches = 2; //official 2/13
            double desiredTicks = determineRightTicks();
            elevatorMotor.set(ControlMode.Position, desiredTicks); //official 2/13 is 5
            System.out.println("desired ticks: " + desiredTicks);
            System.out.println("error: " + (desiredTicks - elevatorMotor.getSelectedSensorPosition()));
            elevatorDeadband(desiredTicks);
        } else if (elevatorState == ElevatorStates.SHELF_ELEVATOR_HEIGHT) {
            desiredInches = 30; //official 2/13
            double desiredTicks = determineRightTicks();
            elevatorMotor.set(ControlMode.Position, desiredTicks); //oficial 2/13 is 39
            elevatorDeadband(desiredTicks);
        } else if (elevatorState == ElevatorStates.MID_ELEVATOR_HEIGHT){
            desiredInches = 40; //official 2/13
            double desiredTicks = determineRightTicks();
            elevatorMotor.set(ControlMode.Position, desiredTicks); //official 2/13
            elevatorDeadband(desiredTicks);
        } else if(elevatorState == ElevatorStates.HIGH_ELEVATOR_HEIGHT){ //high elevator height
            desiredInches = 48; //official 2/13
            double desiredTicks = determineRightTicks();
            elevatorMotor.set(ControlMode.Position, desiredTicks); //change value once we know robot dimensions
            elevatorDeadband(desiredTicks);
        }
        else { //emergency stop again for safety
            elevatorMotor.set(ControlMode.Position, 0);
            elevatorDeadband(0);

        }

        // if(top_limit_switch.get() || bottom_limit_switch.get()){
        //     setState(ElevatorStates.STOP);
        //     System.out.println("Top/bottom limit switch triggered");
        // }
    }
    public double determineRightTicks(){
        return desiredInches * Constants.UNDER_TWO_TICKS_PER_INCH; 
    }


    public void elevatorDeadband(double desiredTicks){
        if (Math.abs(desiredTicks - elevatorMotor.getSelectedSensorPosition()) < deadband){
            elevatorMotor.set(ControlMode.PercentOutput, 0);
            System.out.println("STOPPED");
        }
    }

    public void setState(ElevatorStates newElevatorState){
        elevatorState = newElevatorState;
    }
}
