package frc.robot;

import frc.robot.subsystems.Mechanisms;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PneumaticIntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

public class Buttons {
    
  DrivetrainSubsystem m_drivetrainSubsystem = Robot.m_drivetrainSubsystem;
  PneumaticIntakeSubsystem pneumaticIntakeSubsystem = Robot.m_pneumaticIntakeSubsystem;
  Mechanisms m_mechanisms = Robot.m_mechanisms;
  
  public static int scoringCol = 0;
  private static boolean override = false;
  public static AutoStates level;
  
  public void buttonsPeriodic(){
      if(OI.m_controller.getPOV() >= 225 && OI.m_controller.getPOV() <= 315){
          if (!override){
            System.out.println("dpad 270: left substation");
            level = AutoStates.LEFTPICKUP; 
          }else{
            m_mechanisms.setState(MechanismStates.SHELF);
          }
        }
    
        if(OI.m_controller.getPOV() >= 45 && OI.m_controller.getPOV() <= 135){
          if (!override){
            System.out.println("dpad 90: right substation");
            level = AutoStates.RIGHTPICKUP; 
          }else{
            m_mechanisms.setState(MechanismStates.SHELF);
          }
        }
    
        if(OI.m_controller.getBackButton()){
          //m_VisionSubsystem.setState(VisionSubsystem.VisionStates.DETECTTAPE);
        }
        
        if(OI.m_controller.getAButtonReleased()){
          //pneumaticIntakeSubsystem.setState(PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF);
        }
    
        if(OI.joystick.getRawButton(0)){
          buttonLevel(0);
        } else if (OI.joystick.getRawButton(1)){
          buttonLevel(1);
        } else if (OI.joystick.getRawButton(2)){
          buttonLevel(2);
        } else if (OI.joystick.getRawButton(3)){
          buttonLevel(3);
        } else if (OI.joystick.getRawButton(4)){
          buttonLevel(4);
        } else if (OI.joystick.getRawButton(5)){
          buttonLevel(5);
        } else if (OI.joystick.getRawButton(6)){
          buttonLevel(6);
        } else if (OI.joystick.getRawButton(7)){
          buttonLevel(7);
        } else if (OI.joystick.getRawButton(8)){
          buttonLevel(8);
        }
    
        System.out.println("override is " + override);
    
        //driver
        if (OI.m_controller.getBButton()){ 
          m_drivetrainSubsystem.stopDrive(); //stop all mech?
          m_mechanisms.setState(MechanismStates.HOLDING);
        }
    
        if(OI.m_controller.getXButton()){
        // m_drivetrainSubsystem.pitchBalance(0.0);
        }
    
    
        if(OI.m_controller_two.getAButton()){ 
          if (override){
            System.out.println("xbox: low node");
              m_mechanisms.setState(MechanismStates.LOW_NODE);
            } else {
              level = AutoStates.LOWNODE;
            }
        }
    
        if(OI.m_controller_two.getBButton()){
          if (override){
            System.out.println("xbox: mid");
            m_mechanisms.setState(MechanismStates.MID_NODE);
          }else{
            level = AutoStates.MIDNODE; 
          }
          
        }
    
        if(OI.m_controller_two.getYButton()){
          if (override){
            System.out.println("xbox: high node");
            m_mechanisms.setState(MechanismStates.HIGH_NODE);
          }else{
            level = AutoStates.HIGHNODE; 
          }
    
        }
    
        if(OI.m_controller_two.getXButton()){ //override button
          override = !override;
        }
    
    
    
        if(OI.m_controller_two.getLeftBumperReleased()){ //needs its own button & not enough
          if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING || PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF){
            pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING);
          } else if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING){
            pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING); 
          }
        }
    
        if(OI.m_controller_two.getRightBumper()){
          if(override){
            m_mechanisms.setState(MechanismStates.SHELF);
            System.out.println("xbox: shelf"); 
          }else{
            level = AutoStates.RIGHTPICKUP;
          }
        }
    
        if (OI.m_controller_two.getStartButton()){
          
        }
    
        if (OI.m_controller_two.getBackButton()){
    
        }
        
  }


  public void buttonLevel(int col){
      if(Robot.isBlueAlliance.equalsIgnoreCase("blue")){
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
