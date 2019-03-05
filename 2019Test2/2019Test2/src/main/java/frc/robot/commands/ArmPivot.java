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

public class ArmPivot extends Command {
  private ArmPosition armPosition;
  public ArmPivot(ArmPosition armPosition) {
    this.armPosition = armPosition;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    RobotMap.armMotor.set(ControlMode.PercentOutput, .5);
    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   /* switch(armPosition){
      case _45:
        return RobotMap.armEncoder.getDistance() >= 1;
      case _180:
        return RobotMap.armEncoder.getDistance() >= 1;
      case home:
        return RobotMap.armEncoder.getDistance() <= 1;
      case miss:
        return RobotMap.armEncoder.getDistance() >= 1;
      default:*/
        return true;
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
