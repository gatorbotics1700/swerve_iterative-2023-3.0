package frc.robot;

import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem;
import frc.robot.subsystems.PneumaticArmPivot.PneumaticPivotStates;
import frc.robot.subsystems.PneumaticArmPivot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Buttons {
    
  private DrivetrainSubsystem m_drivetrainSubsystem = Robot.m_drivetrainSubsystem;
  private Mechanisms m_mechanisms = Robot.m_mechanisms;
  private PneumaticIntakeSubsystem pneumaticIntakeSubsystem = m_mechanisms.pneumaticIntakeSubsystem;
  private PneumaticArmPivot armPneumaticPivot = m_mechanisms.armPneumaticPivot;
  
  public void buttonsPeriodic(){

    //codriver
      if(OI.m_controller_two.getYButton()){ 
        System.out.println("xbox: shelf");
        m_mechanisms.setState(MechanismStates.SHELF);
      }
      if(OI.m_controller_two.getAButton()){ 
          System.out.println("xbox: low node");
          m_mechanisms.setState(MechanismStates.LOW_NODE);
      }
    
      if(OI.m_controller_two.getBButton()){
          System.out.println("xbox: mid");
          m_mechanisms.setState(MechanismStates.MID_NODE); 
      }
    
      if(OI.m_controller_two.getXButtonPressed()){ //used to be ground pickup button
      
      }
    
      if(OI.m_controller_two.getLeftBumperReleased()){ 
        System.out.println("intake");
        if(pneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.PINCHING || 
           pneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF){
          pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.RELEASING);
        } else if(pneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.RELEASING){
          pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.PINCHING); 
        }
      }
    
      // if(OI.m_controller_two.getPOV() > 135 && OI.m_controller_two.getPOV() < 225){ //new pivot - south dpad

      if(OI.m_controller_two.getRightBumperReleased()){ 
        System.out.println("xbox: single substation");
        m_mechanisms.setState(MechanismStates.SUB);
      }
  
      if (OI.m_controller_two.getBackButton()){
        //this is the center button on left with 2 squares
        System.out.println("back button: holding");
        m_mechanisms.setState(MechanismStates.HOLDING);
      }
      
      if(OI.m_controller_two.getPOV() > 45 && OI.m_controller_two.getPOV() < 135){
        m_mechanisms.setState(MechanismStates.MANUAL_TELESCOPE);
        System.out.println("dpad: manual telescope");
      }

      if(OI.m_controller_two.getPOV() > 225 && OI.m_controller_two.getPOV() < 315){
        m_mechanisms.setState(MechanismStates.MANUAL_ELEVATOR);
        System.out.println("dpad: manual elevator");
      }

      //driver
      if (OI.m_controller.getBButton()){ //emergency stop EVERYTHING
        m_drivetrainSubsystem.stopDrive(); 
        m_mechanisms.setState(MechanismStates.HOLDING);
      }
    
      if(OI.m_controller.getXButton()){
        m_drivetrainSubsystem.pitchBalance(0.0);
      }

        // System.out.println("pivot");
        // if(armPneumaticPivot.pneumaticPivotState==PneumaticPivotStates.DOWN 
        // || armPneumaticPivot.pneumaticPivotState==PneumaticPivotStates.OFF){
        //   armPneumaticPivot.setState(PneumaticPivotStates.UP);
        // } else if (armPneumaticPivot.pneumaticPivotState==PneumaticPivotStates.UP){
        //   armPneumaticPivot.setState(PneumaticPivotStates.DOWN);
        // }
  }
}
