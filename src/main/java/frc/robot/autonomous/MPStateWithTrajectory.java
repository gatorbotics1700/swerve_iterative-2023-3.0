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
        TRAJECTORY, //MP version of PD states DRIVE and FASTDRIVE // note: don't need FASTDRIVE in MP bc MP doesn't have the same problems as PD
        MID, //PD version = MIDNODE
        ENGAGE, //PD version = ENGAGE
        STOP, 
        LOW, //PD version = LOWNODE
        FIRST;
    } 




}
