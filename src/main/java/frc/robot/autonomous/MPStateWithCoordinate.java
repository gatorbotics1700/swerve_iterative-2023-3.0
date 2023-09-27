package frc.robot.autonomous;

import java.lang.Thread.State;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;

public class MPStateWithCoordinate{

    public final MPStates mpState;
    public final Pose2d coordinate;

    public MPStateWithCoordinate(MPStates mpStates, Pose2d coordinate){
        this.mpState = mpStates;
        this.coordinate = coordinate;
    }

    public MPStateWithCoordinate(MPStates mpStates){
        this.mpState = mpStates;
        this.coordinate = null;
    }

    public static enum MPStates{
        TRAJECTORY, // anaika notes:fix
        MID, 
        BALANCE,
        STOP,
        LOW,
        FIRST;
    } 




}
