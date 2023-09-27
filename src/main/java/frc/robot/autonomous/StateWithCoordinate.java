package frc.robot.autonomous;

import java.lang.Thread.State;

import edu.wpi.first.math.geometry.Pose2d;

public class StateWithCoordinate{

    public final AutoStates autoState;
    public final Pose2d coordinate;

    public StateWithCoordinate(AutoStates autoStates, Pose2d coordinate){
        this.autoState = autoStates;
        this.coordinate = coordinate;
    }

    public StateWithCoordinate(AutoStates autoStates){
        this.autoState = autoStates;
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
        ENGAGE,
        FASTDRIVE,
        PICKUP;
    } 




}
