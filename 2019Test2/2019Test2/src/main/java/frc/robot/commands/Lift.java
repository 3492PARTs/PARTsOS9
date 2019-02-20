/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

enum LiftLevel{
  Low, Middle, High;
}


public class Lift extends Command {

  private LiftLevel liftLevel;
  private double distance;

  public Lift(LiftLevel liftLevel) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.liftLevel = liftLevel; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    switch(liftLevel){
      case Low: 
        distance = 0;
      break;

      case Middle:
        distance = 1; //TODO: change values
      break;

      case High:
        distance = 2;
      break;

      default:
      distance = 0;
      break;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (RobotMap.liftEncoder.getDistance() > distance){
      RobotMap.liftMotor.set(ControlMode.PercentOutput, -0.5);
    }
    else{
      RobotMap.liftMotor.set(ControlMode.PercentOutput, 0.5);
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return RobotMap.liftMotor.getSelectedSensorPosition() < (distance + 0.5) || RobotMap.liftMotor.getSelectedSensorPosition() > (distance - 0.5);
  //change the 0.5
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
