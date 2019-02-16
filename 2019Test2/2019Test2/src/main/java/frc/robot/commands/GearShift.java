/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

/**
 * An example command.  You can replace me with your own command.
 */
public class GearShift extends Command {
    private OI m_oi;
  public GearShift() {
      m_oi = new OI();
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if (RobotMap.encoder0.getVelocity() >= 1500){
          //solenoid to fire
      }
      else if (m_oi.driveStick.getRawAxis(1) < 0.01){
          //shift low gear

      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
