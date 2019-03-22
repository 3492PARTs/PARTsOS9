/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

/**
 * An example command.  You can replace me with your own command.
 */
public class Arm90 extends Command {
 private double distance;
 private double startPositionArm90;
 private  boolean up = false, down = false;
  public Arm90(double startPositionArm90) {
      this.startPositionArm90 = startPositionArm90;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_subsystem);
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    distance = 1632;
    if ((RobotMap.armMotor.getSelectedSensorPosition() - startPositionArm90) > distance){
        up = true;
    }
    else{
        down = true;
    }
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if ((RobotMap.armMotor.getSelectedSensorPosition() - startPositionArm90) > distance){
        RobotMap.armMotor.set(ControlMode.PercentOutput, .5);
    }
    else{
        RobotMap.armMotor.set(ControlMode.PercentOutput, -.5);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(up && (RobotMap.armMotor.getSelectedSensorPosition() - startPositionArm90) < distance) return true;
    if (down && (RobotMap.armMotor.getSelectedSensorPosition() - startPositionArm90) > distance) return true;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
      RobotMap.armMotor.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
