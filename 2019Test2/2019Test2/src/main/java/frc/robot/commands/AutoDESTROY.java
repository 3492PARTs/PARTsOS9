/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.Encoder;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.Command;

public class AutoDESTROY extends Command {
  private LimeLight limeLight;
  private double KpDistance;
  private Encoder encoder;
  private double rotations;


  public AutoDESTROY(double rotations) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    limeLight = new LimeLight(); 
    encoder = new Encoder(); 
    this.rotations = rotations; 

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    
    KpDistance = -0.1;
    double leftCommand = 0;
    double rightCommand = 0;
    double distanceAdjust = 0;
    double KpAim = -0.1;
    double minAimCommand = 0.05;
    double headingError = -limeLight.getX();
    double distanceError = -limeLight.getY();
    double steeringAdjust = 0.0;
    
    if(limeLight.getX() > 1.0){
      steeringAdjust = KpAim * headingError - minAimCommand;
    }
    else if(limeLight.getX() < -1.0){
    
    steeringAdjust = KpAim * headingError + minAimCommand; 
    }
    distanceAdjust = KpDistance * distanceError;
    leftCommand += steeringAdjust + distanceAdjust;
    rightCommand -= steeringAdjust + distanceAdjust;
    //The multiplier for the speed here does NOT make the actual speed go down by that much.

    RobotMap.dDrive.tankDrive((-leftCommand)*.3,(rightCommand)*.3);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (RobotMap.frontLeftMotor.get() == 0 && 
    RobotMap.backLeftMotor.get() == 0 && 
    RobotMap.frontRightMotor.get() == 0 && 
    RobotMap.backRightMotor.get() == 0) 
    ||
    (encoder.getEncoderValue0() >= rotations && 
    encoder.getEncoderValue1() >= rotations);
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
