/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Encoder;

public class AutoTurn extends Command {
  private double direction;
  private Encoder encoder;
  private double distance;
  public AutoTurn(double direction, double distance) {
    this.direction = direction;
    encoder = new Encoder();
    this.distance = distance;
  }

  @Override
  protected void initialize() {
    encoder.resetAll();
  }

  @Override
  protected void execute() {
    RobotMap.dDrive.arcadeDrive(0, .3*direction);
  }

  @Override
  protected boolean isFinished() {
    return Math.abs(encoder.getEncoder0Distance()) >= distance && Math.abs(encoder.getEncoder1Distance()) >= distance;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}