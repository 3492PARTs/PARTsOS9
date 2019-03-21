/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;




public class Lift extends Command {

  private LiftLevel liftLevel;
  private double distance, startPosition, distanceToTravel, posAtButtonPress;
  private  boolean up = false, down = false;
  private OI m_oi = new OI();

  public Lift(LiftLevel liftLevel ,double startPosition) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.liftLevel = liftLevel; 
    this.startPosition = startPosition;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    switch(liftLevel){
      case Low: 
        distance = -1500;//startPosition- need to make command for it
      break;

      case Middle:
        distance = -35406; //TODO: change values
      break;

      case High:
        distance = -71470;
      break;

      default:
      distance = -1500;
      break;
    }

    if ((RobotMap.liftMotor.getSelectedSensorPosition() - startPosition) > distance){
      up = true; // up
    }
    else{
      down = true; // down
    }

    posAtButtonPress = (RobotMap.liftMotor.getSelectedSensorPosition() - startPosition);
    distanceToTravel = Math.abs(distance - (RobotMap.liftMotor.getSelectedSensorPosition() - startPosition));

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if ((RobotMap.liftMotor.getSelectedSensorPosition() - startPosition) > distance){
      if ((Math.abs(RobotMap.liftMotor.getSelectedSensorPosition() - posAtButtonPress) >= (distanceToTravel*.75))
          ||
          (Math.abs(RobotMap.liftMotor.getSelectedSensorPosition() - posAtButtonPress) <= (distanceToTravel*.25))
          ||
          (distanceToTravel <= 10000)){
        RobotMap.liftMotor.set(ControlMode.PercentOutput, -.3);
      }
      else{
        RobotMap.liftMotor.set(ControlMode.PercentOutput, -.7); // test with -.7, if works, put back as -1
      }
    }
    else{
      if ((Math.abs(RobotMap.liftMotor.getSelectedSensorPosition() - posAtButtonPress) >= (distanceToTravel*.75))
          ||
          (Math.abs(RobotMap.liftMotor.getSelectedSensorPosition() - posAtButtonPress) <= (distanceToTravel*.25))
          ||
          (distanceToTravel <= 10000)){
        RobotMap.liftMotor.set(ControlMode.PercentOutput, .3);
      }
      else{
        RobotMap.liftMotor.set(ControlMode.PercentOutput, .7);  // test with .7, if works, put back as 1
    
      }

    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    System.out.println("running lift: going to " + distance + " currently at: " + RobotMap.liftMotor.getSelectedSensorPosition());
    if(up && (RobotMap.liftMotor.getSelectedSensorPosition() -startPosition) < distance) return true;
    if (down && (RobotMap.liftMotor.getSelectedSensorPosition() - startPosition) > distance) return true;
    if(m_oi.launchPad.getRawButton(6)) return true;
    return false;
    //change the 0.5
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("end lift");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
