/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.LimeLight;

public class AutoSeekAimDestroy extends Command {
  private LimeLight limeLight;
  private double seekDirection; 
  private double destroyRotations;
  private AutoSeekAimDestroyCmdGrp autoSeekAimDestroyCmdGrp;

  public AutoSeekAimDestroy(double seekDirection, double destroyRotations) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    limeLight = new LimeLight();
    this.seekDirection = seekDirection;
    this.destroyRotations = destroyRotations;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    autoSeekAimDestroyCmdGrp = new AutoSeekAimDestroyCmdGrp(seekDirection, destroyRotations);
    autoSeekAimDestroyCmdGrp.start();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return limeLight.getX() > -1 && limeLight.getX() < 1;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.dDrive.arcadeDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}