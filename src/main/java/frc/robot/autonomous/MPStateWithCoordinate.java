package frc.robot.autonomous;

import java.lang.Thread.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;

public class MPStateWithCoordinate{

    public final MPStates mpState;
    public final Trajectory trajectory;

    public MPStateWithCoordinate(MPStates mpStates, Trajectory trajectory){
        this.mpState = mpStates;
        this.trajectory = trajectory;
    }

    public MPStateWithCoordinate(MPStates mpStates){
        this.mpState = mpStates;
        this.trajectory = null;
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
