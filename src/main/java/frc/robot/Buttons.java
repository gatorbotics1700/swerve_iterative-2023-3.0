package frc.robot;

import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class Buttons {
    
  private DrivetrainSubsystem m_drivetrainSubsystem = Robot.m_drivetrainSubsystem;
  private Mechanisms m_mechanisms = Robot.m_mechanisms;
  private LauncherSubsystem m_launcher = m_mechanisms.launcherSubsystem;
  //private PneumaticArmPivot armPneumaticPivot = m_mechanisms.armPneumaticPivot;
  
  public void buttonsPeriodic(){

    //codriver
      if(OI.m_controller_two.getYButton()){  
        if(m_launcher.speed <= 0.95) {
          m_launcher.speed += 0.05;
        } else {
          System.out.println("exceeded limit");
        }
        System.out.println("speed:" + m_launcher.speed);
      }

      if(OI.m_controller_two.getAButton()){ 
          if(m_launcher.speed >= -0.95) {
            m_launcher.speed -= 0.05;
          } else {
            System.out.println("exceeded limit");
          }
          System.out.println("speed:" + m_launcher.speed);
      }
    
      if(OI.m_controller_two.getBButton()){ //feed
        m_launcher.isRunning = true;
        m_launcher.speed = -0.5;
        System.out.println("launcher intaking!!");
      }
    
      if(OI.m_controller_two.getXButtonPressed()){ //launch
        m_launcher.isRunning = true;
        m_launcher.speed = 0.5;
        System.out.println("launching!!");
      }
    
      if(OI.m_controller_two.getLeftBumperReleased()){ 
        System.out.println("intake");
      }
    
      // if(OI.m_controller_two.getPOV() > 135 && OI.m_controller_two.getPOV() < 225){ //new pivot - south dpad

      if(OI.m_controller_two.getRightBumperReleased()){ 
        m_launcher.isRunning = false;
        System.out.println("stopping!");
      }
  
      if (OI.m_controller_two.getBackButton()){
        //this is the center button on left with 2 squares
      }
      
      if(OI.m_controller_two.getPOV() > 45 && OI.m_controller_two.getPOV() < 135){
      }

      if(OI.m_controller_two.getPOV() > 225 && OI.m_controller_two.getPOV() < 315){
      }

      //driver
      if (OI.m_controller.getBButton()){ //emergency stop EVERYTHING
        m_drivetrainSubsystem.stopDrive(); 
      }
    
      if(OI.m_controller.getXButton()){
        m_drivetrainSubsystem.pitchBalance(0.0);
      }

      if(OI.m_controller.getAButton()){
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
