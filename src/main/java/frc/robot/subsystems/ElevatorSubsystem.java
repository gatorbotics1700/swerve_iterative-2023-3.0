package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.OI;
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
    private final double HIGHHEIGHT = 33; 
    private final double MIDHEIGHT = 33; //changed 3/14 245pm
    private final double LOWHEIGHT = 10; //old was 30; 
    private final double SHELF = 33; 
    private final double MAX_HEIGHT = 33;



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
        HIGH_ELEVATOR_HEIGHT,
        MANUAL,
        STOPPED;
    }

    public void init(){
        System.out.println("elevator init!!!!");
        elevatorMotor.setInverted(true); // looking from the front of the robot, clockwise is false (:
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        setState(ElevatorStates.STOPPED);
        //elevatorMotor.setSelectedSensorPosition(0);
        //configuring deadband
        elevatorMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		/* Config Position Closed Loop gains in slot0, typically kF stays zero. */
		elevatorMotor.config_kP(Constants.kPIDLoopIdx, elevatorGains.kP, Constants.kTimeoutMs);
		elevatorMotor.config_kI(Constants.kPIDLoopIdx, elevatorGains.kI, Constants.kTimeoutMs);
		elevatorMotor.config_kD(Constants.kPIDLoopIdx, elevatorGains.kD, Constants.kTimeoutMs);
    }

    /*
     * went 4 when we said 5
     * 4 on here means bottom of intake is 8 above ground
     * add node safety with 5?
     */
    public void periodic(){
        //System.out.println("current elevator motor position:" + elevatorMotor.getSelectedSensorPosition()/Constants.TICKS_PER_INCH);
        if (elevatorState == ElevatorStates.ZERO){ //emergency stop
            System.out.println("desired ticks: 0");
            System.out.println("error: " + (0 - elevatorMotor.getSelectedSensorPosition()));
            elevatorDeadband(0);
        } else if (elevatorState == ElevatorStates.LOW_ELEVATOR_HEIGHT){
            desiredInches = 10; //official 2/13 //5 went to 8 above, so safety for node adds 5 more
            double desiredTicks = determineRightTicks();
            System.out.println("desired ticks: " + desiredTicks);
            System.out.println("error: " + (desiredTicks - elevatorMotor.getSelectedSensorPosition()));
            elevatorDeadband(desiredTicks);
        } else if (elevatorState == ElevatorStates.SHELF_ELEVATOR_HEIGHT) {
            desiredInches = 25; //official 2/13
            double desiredTicks = determineRightTicks();
            elevatorDeadband(desiredTicks);
        } else if (elevatorState == ElevatorStates.MID_ELEVATOR_HEIGHT){
            desiredInches = 34; //official 2/13 //went 22 inches
            double desiredTicks = determineRightTicks();
            elevatorDeadband(desiredTicks);
        } else if(elevatorState == ElevatorStates.HIGH_ELEVATOR_HEIGHT){ //high elevator height
            desiredInches = 34; //48 - 15; //official 2/13
            double desiredTicks = determineRightTicks();
            elevatorDeadband(desiredTicks);
        }else if (elevatorState == ElevatorStates.MANUAL){
            manual();
        } else { //emergency stop again for safety
            elevatorMotor.set(ControlMode.PercentOutput, 0.0);
        }

        // if(top_limit_switch.get() || bottom_limit_switch.get()){
        //     setState(ElevatorStates.STOP);
        //     System.out.println("Top/bottom limit switch triggered");
        // }
    }
    public double determineRightTicks(){
        return desiredInches * Constants.ELEVATOR_TICKS_PER_INCH; 
    }


    public void elevatorDeadband(double desiredTicks){
        if (Math.abs(desiredTicks - elevatorMotor.getSelectedSensorPosition()) > deadband){
            elevatorMotor.set(ControlMode.Position, desiredTicks); //official 2/13 is 5
        } else {
            elevatorMotor.set(ControlMode.PercentOutput, 0);
            //System.out.println("ELEVATOR STOPPED");
        }
    }

    public void setState(ElevatorStates newElevatorState){
        elevatorState = newElevatorState;
    }

    public void manual(){
        if(elevatorMotor.getSelectedSensorPosition()*Constants.ELEVATOR_TICKS_PER_INCH >= MAX_HEIGHT){
            if(OI.getRightAxis() > 0.2){
                elevatorMotor.set(ControlMode.PercentOutput, 0.2);
            }else if(OI.getRightAxis() < -0.2) {
                elevatorMotor.set(ControlMode.PercentOutput, -0.2);
            }
        }
    }

    public boolean isAtHigh(){
        if(Math.abs(elevatorMotor.getSelectedSensorPosition()-HIGHHEIGHT*Constants.ELEVATOR_TICKS_PER_INCH)<3*Constants.ELEVATOR_TICKS_PER_INCH){
            return true; 
        }
        return false; 
    }

    public boolean isAtMid(){
        if(Math.abs(elevatorMotor.getSelectedSensorPosition()-MIDHEIGHT*Constants.ELEVATOR_TICKS_PER_INCH)<3*Constants.ELEVATOR_TICKS_PER_INCH){
            return true; 
        }
        return false; 
    }

    public boolean isAtLow(){
        if(Math.abs(elevatorMotor.getSelectedSensorPosition()-LOWHEIGHT*Constants.ELEVATOR_TICKS_PER_INCH)<3*Constants.ELEVATOR_TICKS_PER_INCH){
            return true; 
        }
        return false; 
    }

    public boolean isAtShelf(){
        if(Math.abs(elevatorMotor.getSelectedSensorPosition() -SHELF) < 3 * Constants.SWERVE_TICKS_PER_INCH){
            return true;
        }
        return false;
    }

}
