/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

public class DoNothing extends Command {
  private long time;
  public DoNothing() {
  }

  @Override
  protected void initialize() {
    time = System.currentTimeMillis() + 5000;
  }

  @Override
  protected void execute() {
    RobotMap.dDrive.arcadeDrive(0, 0);
    System.out.println("Do nothing");
  }

  @Override
  protected boolean isFinished() {
    return System.currentTimeMillis() >= time;
  }

  @Override
  protected void end() {
    System.out.println("Do nothing done");
  }

  @Override
  protected void interrupted() {
  }
}
