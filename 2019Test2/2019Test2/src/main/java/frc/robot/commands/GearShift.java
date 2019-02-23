/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class GearShift extends Command {
    private OI m_oi;
  public GearShift() {
      m_oi = new OI();
    requires(Robot.m_subsystem);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
      if (Math.abs(RobotMap.encoder0.getVelocity()) >= 1500){
          RobotMap.solenoidStan.set(DoubleSolenoid.Value.kForward);
      }
      else if (m_oi.driveStick.getRawAxis(1) < 0.01){
          RobotMap.solenoidStan.set(DoubleSolenoid.Value.kReverse);
      }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
