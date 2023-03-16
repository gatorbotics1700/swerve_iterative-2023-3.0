package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;

public class StateWithCoordinate{

    public final AutoStates state;
    public final Pose2d coordinate;

    public StateWithCoordinate(AutoStates state, Pose2d coordinate){
        this.state = state;
        this.coordinate = coordinate;
    }

    public StateWithCoordinate(AutoStates state){
        this.state = state;
        this.coordinate = null;
    }

    public static enum AutoStates{
        
        FIRST,
        DRIVE,
        STOP,
        HIGHNODE,
        MIDNODE,
        LOWNODE,
        FIRSTHIGHNODE,
        INTAKING,
        OUTTAKING,
        BALANCING,
        LEFTPICKUP,
        RIGHTPICKUP;
    } 




}
