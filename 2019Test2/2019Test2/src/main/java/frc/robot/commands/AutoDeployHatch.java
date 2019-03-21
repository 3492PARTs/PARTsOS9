/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoDeployHatch extends Command {
private long exeTime;
  public AutoDeployHatch() {
  }

  @Override
  protected void initialize() {
    exeTime = System.currentTimeMillis();
  }

  @Override
  protected void execute() {
    //RobotMap.solenoidSteve.set(true);
  }

  @Override
  protected boolean isFinished() {
    return System.currentTimeMillis() >= exeTime + 1000;
  }

  @Override
  protected void end() {    
    System.out.println("hatch deployed");
    //RobotMap.solenoidSteve.set(true);
  }

  @Override
  protected void interrupted() {
  }
}
