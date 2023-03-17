package frc.robot;

import frc.robot.subsystems.Mechanisms;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PneumaticIntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Vision.AprilTagSubsystem;
import frc.robot.subsystems.Vision.AprilTagSubsystem.AprilTagSequence;

public class Buttons {
    
  private DrivetrainSubsystem m_drivetrainSubsystem = Robot.m_drivetrainSubsystem;
  private PneumaticIntakeSubsystem pneumaticIntakeSubsystem = Robot.m_pneumaticIntakeSubsystem;
  private Mechanisms m_mechanisms = Robot.m_mechanisms;
  private AprilTagSubsystem m_AprilTagSubsystem = Robot.m_aprilTagSubsystem;
  
  public static int scoringCol = 0;
  private static boolean override = false;
  public static AutoStates level;
  
  public void buttonsPeriodic(){
      if(OI.m_controller_two.getPOV() >= 225 && OI.m_controller_two.getPOV() <= 315){
          if (override == false){
            System.out.println("dpad 270: left substation");
            level = AutoStates.LEFTPICKUP;
            if(Robot.isBlueAlliance){
              scoringCol = 36;
            }else{
              scoringCol = 38;
            }
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
          }else{
            m_mechanisms.setState(MechanismStates.SHELF);
          }
        }
    
        if(OI.m_controller_two.getPOV() >= 45 && OI.m_controller_two.getPOV() <= 135){
          if (override == false){
            System.out.println("dpad 90: right substation");
            level = AutoStates.RIGHTPICKUP;
            if(Robot.isBlueAlliance){
              scoringCol = 37;
            }else{
              scoringCol = 39;
            }
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
          }else{
            m_mechanisms.setState(MechanismStates.SHELF);
          }
        }

        if(OI.m_controller_two.getAButton()){
          if(override){
            m_mechanisms.setState(MechanismStates.LOW_NODE);
          }else{
            level = AutoStates.LOWNODE;
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
          }
        }

        if(OI.m_controller_two.getBButton()){
          if(override){
            m_mechanisms.setState(MechanismStates.MID_NODE);
          }else{
            level = AutoStates.MIDNODE;
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
          }
        }

        if(OI.m_controller_two.getYButton()){
          if(override){
            m_mechanisms.setState(MechanismStates.HIGH_NODE);
          }else{
            level = AutoStates.HIGHNODE;
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
          }
        }

        if(OI.m_controller_two.getXButton()){
          if(override){
            m_mechanisms.setState(MechanismStates.GROUNDPICKUP);
          }else{
            level = AutoStates.INTAKING;
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
          }
        }

        if(OI.m_controller_two.getLeftBumperReleased()){
          if(pneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING || PneumaticIntakeSubsystem.pneumaticIntakeState == PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF){
            pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeStates.RETRACTING);
          }else if(pneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING){
            pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeStates.ACTUATING);
          }
        }

        if(OI.m_controller_two.getStartButton()){
          override = !override;
        }

        if(Math.abs(OI.getLeftAxis()) > 0.2){
          m_mechanisms.setState(MechanismStates.MANUAL_TELESCOPE);
        }

        if(Math.abs(OI.getRightAxis()) > 0.2){
          m_mechanisms.setState(MechanismStates.MANUAL_ELEVATOR);
        }

        //driver

        if(OI.m_controller.getBButton()){
          m_drivetrainSubsystem.stopDrive();
          m_mechanisms.setState(MechanismStates.HOLDING);
          m_AprilTagSubsystem.setState(AprilTagSequence.OFF);
        }

        if(OI.m_controller.getXButton()){
          m_drivetrainSubsystem.pitchBalance(0.0);
        }
    
        if(OI.m_controller.getBackButton()){
          //m_VisionSubsystem.setState(VisionSubsystem.VisionStates.DETECTTAPE);
        }
        
        if(OI.m_controller.getAButtonReleased()){
          //pneumaticIntakeSubsystem.setState(PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF);
        }

        for (int i=1; i < 10; i++){
          if(OI.joystick.getRawButton(i)){
            buttonLevel(i-1);
          }
        }
      }


  public void buttonLevel(int col){
      if(Robot.isBlueAlliance){
        if(level == AutoStates.LOWNODE){
          scoringCol = col + 9;
        } else {
          scoringCol = col;
        }
      } else { //red
        if(level == AutoStates.LOWNODE){
          scoringCol = col + 27;
        } else {
          scoringCol = col + 18;
        }
      }
    }

}
