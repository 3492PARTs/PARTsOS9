/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;
import frc.robot.subsystems.Encoders;

public class AutoDriveStraight extends Command {
  private Encoders encoder;
  private double distance;
  private double direction;
  public AutoDriveStraight(double distance, double direction) {
    requires(Robot.m_subsystem);
    this.distance = distance;
    this.direction = direction;
    encoder = new Encoders(); 
  }

  @Override
  protected void initialize() {
    encoder.resetAll();
  }

  @Override
  protected void execute() {
    RobotMap.dDrive.tankDrive(1 * direction, 1 * direction);
  }

  @Override
  protected boolean isFinished() {
    return (Math.abs(encoder.getEncoder0Distance()) >= distance && Math.abs(encoder.getEncoder1Distance())>= distance);
  }

  @Override
  protected void end() {
    RobotMap.dDrive.arcadeDrive(0,0);
    System.out.println("finish drv frd");
    System.out.println("Auto drv frd dist " + encoder.getEncoder0Distance());
  }

  @Override
  protected void interrupted() {
  }
}
