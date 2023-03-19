package frc.robot;

import frc.robot.subsystems.Mechanisms;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PneumaticIntakeSubsystem;
//import frc.robot.subsystems.ArmPneumaticPivot.PneumaticPivotStates;
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
    System.out.println("OVERRIDE: " + override);

    //codriver
      if(OI.m_controller_two.getPOV() >= 225 && OI.m_controller_two.getPOV() <= 315){
          if (!override){
            System.out.println("dpad 270: left substation");
            level = AutoStates.LEFTPICKUP; 
            if (Robot.isBlueAlliance){
              scoringCol = 36;
            } else {
              scoringCol = 38;
            }
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
          }else{
            m_mechanisms.setState(MechanismStates.SHELF);
          }
          System.out.println("hi: " + level);
        }
    
        if(OI.m_controller_two.getPOV() >= 45 && OI.m_controller_two.getPOV() <= 135){
          if (!override){
            System.out.println("dpad 90: right substation");
            level = AutoStates.RIGHTPICKUP; 
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
            if (Robot.isBlueAlliance){
              scoringCol = 37;
            } else {
              scoringCol = 39;
            }
            // System.out.println("level: " + level);
          }else{
            m_mechanisms.setState(MechanismStates.SHELF);
          }
          System.out.println("hello: " + level);
        }

        if(OI.m_controller_two.getAButton()){ 
          if (override){
            System.out.println("xbox: low node");
            m_mechanisms.setState(MechanismStates.LOW_NODE);
            } else {
              level = AutoStates.LOWNODE;
              m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
              // System.out.println("level: " + level);
            }
        }
    
        if(OI.m_controller_two.getBButton()){
          if (override){
            System.out.println("xbox: mid");
            m_mechanisms.setState(MechanismStates.MID_NODE);
          }else{
            level = AutoStates.MIDNODE; 
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
            // System.out.println("level: " + level);
          }
          
        }
    
        if(OI.m_controller_two.getYButton()){
          if (override){
            System.out.println("xbox: high node");
            m_mechanisms.setState(MechanismStates.HIGH_NODE);
          }else{
            level = AutoStates.HIGHNODE; 
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
            // System.out.println("level: " + level);
          }
    
        }
    
        if(OI.m_controller_two.getXButtonPressed()){ //override button
          if(override){
            m_mechanisms.setState(MechanismStates.GROUNDPICKUP);
          } else {
            level = AutoStates.INTAKING;
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
            // System.out.println("level: " + level);
          }
        }
    
        /*if(OI.m_controller_two.getLeftBumperReleased()){ //needs its own button & not enough
          System.out.println("intake");
          if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING || PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF){
            pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING);
          } else if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING){
            pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING); 
          }
        }*/
    
        if(OI.m_controller_two.getRightBumper()){
          if(override){
            m_mechanisms.setState(MechanismStates.SHELF);
            System.out.println("xbox: shelf"); 
          }else{
            level = AutoStates.RIGHTPICKUP;
            if (Robot.isBlueAlliance){
              scoringCol = 37;
            } else {
              scoringCol = 39;
            }
            m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
            // System.out.println("level: " + level);
          }
        }
    
        if (OI.m_controller_two.getStartButton()){
          //this is the center button on the right with 3 lines
          override = !override;
          System.out.println("override is " + override);
        }
    
        if (OI.m_controller_two.getBackButton()){
          //this is the center button on left with 2 squares
          m_mechanisms.setState(MechanismStates.HOLDING);
          //System.out.println("BACK BUTTON SAYS HEY");
        }
        
        if(Math.abs(OI.m_controller_two.getLeftY()) > 0.2){
          m_mechanisms.setState(MechanismStates.MANUAL_TELESCOPE);
        }

        if(Math.abs(OI.m_controller_two.getRightY()) > 0.2){
          m_mechanisms.setState(MechanismStates.MANUAL_ELEVATOR);
        }

        //driver
        if (OI.m_controller.getBButton()){ 
          m_drivetrainSubsystem.stopDrive(); //stop all mech?
          m_mechanisms.setState(MechanismStates.HOLDING);
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

        for (int i = 1; i < 10; i++){
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
      //m_AprilTagSubsystem.setState(AprilTagSequence.DETECT);
      }
      System.out.println("Scoring Col: " + scoringCol);
    }
   

}
