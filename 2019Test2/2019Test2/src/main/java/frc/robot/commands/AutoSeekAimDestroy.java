/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.LimeLight;

public class AutoSeekAimDestroy extends Command {
  private LimeLight limeLight;
  private double seekDirection; 
  private double destroyDistance;
  private AutoSeekAimDestroyCmdGrp autoSeekAimDestroyCmdGrp;
  private Encoders encoder = new Encoders();
  public AutoSeekAimDestroy(double seekDirection, double destroyDistance) {
    limeLight = new LimeLight();  
    this.seekDirection = seekDirection;
    this.destroyDistance = destroyDistance;
  }

  @Override
  protected void initialize() {
    encoder.resetAll();
  }

  @Override
  protected void execute() {
    autoSeekAimDestroyCmdGrp = new AutoSeekAimDestroyCmdGrp(seekDirection, destroyDistance);
    autoSeekAimDestroyCmdGrp.start();
  }

  @Override
  protected boolean isFinished() {
    return (Math.abs(RobotMap.frontLeftMotor.get()) <= .01 && 
    Math.abs(RobotMap.backLeftMotor.get()) <= .01 && 
    Math.abs(RobotMap.frontRightMotor.get()) <= .01 && 
    Math.abs(RobotMap.backRightMotor.get()) <= .01);
  }

  @Override
  protected void end() {
    RobotMap.dDrive.arcadeDrive(0, 0);
    System.out.println("auto seek aim dest grp done");
    System.out.println("seek aim destrou grp dist: " + encoder.getEncoder0Distance());
  }

  @Override
  protected void interrupted() {
  }
}
