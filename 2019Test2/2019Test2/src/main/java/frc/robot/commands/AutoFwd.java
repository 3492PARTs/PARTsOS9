/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;
import frc.robot.subsystems.Encoder;


public class AutoFwd extends Command {

  private Encoder encoder;
  private double rotations;

  public AutoFwd(double rotations) {
    // Use requires() here to declare subsystem dependencies
    // eg. require = s(chassis);
    requires(Robot.m_subsystem);
    this.rotations = rotations;
    encoder = new Encoder(); 

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    encoder.resetAll();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    RobotMap.dDrive.arcadeDrive(0.6,0);
     
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (encoder.getEncoderValue0() < rotations && encoder.getEncoderValue1() < rotations);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.dDrive.arcadeDrive(0,0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
