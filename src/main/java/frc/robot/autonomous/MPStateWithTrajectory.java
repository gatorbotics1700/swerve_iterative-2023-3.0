package frc.robot.autonomous;

import java.lang.Thread.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;

public class MPStateWithTrajectory{

    public final MPStates mpState;
    public final Trajectory trajectory;

    public MPStateWithTrajectory(MPStates mpStates, Trajectory trajectory){
        this.mpState = mpStates;
        this.trajectory = trajectory;
    }

    public MPStateWithTrajectory(MPStates mpStates){
        this.mpState = mpStates;
        this.trajectory = null;
    }

    public static enum MPStates{
        TRAJECTORY, // anaika notes:fix
        MID,        //Avery notes: :(
        BALANCE,
        STOP,
        LOW,
        FIRST;
    } 




}
