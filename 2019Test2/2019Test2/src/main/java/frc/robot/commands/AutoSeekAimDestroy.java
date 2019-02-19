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
import frc.robot.subsystems.LimeLight;

public class AutoSeekAimDestroy extends Command {
  private LimeLight limeLight;
  private double seekDirection; 
  private double destroyDistance;
  private AutoSeekAimDestroyCmdGrp autoSeekAimDestroyCmdGrp;
  private Encoder encoder = new Encoder();
  public AutoSeekAimDestroy(double seekDirection, double destroyDistance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    limeLight = new LimeLight();
    
    this.seekDirection = seekDirection;
    this.destroyDistance = destroyDistance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    encoder.resetAll();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    autoSeekAimDestroyCmdGrp = new AutoSeekAimDestroyCmdGrp(seekDirection, destroyDistance);
    autoSeekAimDestroyCmdGrp.start();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(RobotMap.frontLeftMotor.get()) <= .01 && 
    Math.abs(RobotMap.backLeftMotor.get()) <= .01 && 
    Math.abs(RobotMap.frontRightMotor.get()) <= .01 && 
    Math.abs(RobotMap.backRightMotor.get()) <= .01);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.dDrive.arcadeDrive(0, 0);
    System.out.println("auto seek aim dest grp done");
    System.out.println("seek aim destrou grp dist: " + encoder.getEncoder0Distance());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
