/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoDESTROY extends Command {
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  double x,y,area,v;
  double KpDistance;

  public AutoDESTROY() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  tx = table.getEntry("tx");
  ty = table.getEntry("ty");
  ta = table.getEntry("ta");
  tv = table.getEntry("tv");
  
 
   
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    v = tv.getDouble(0.0);

    KpDistance = -0.1;
    double leftCommand = 0;
    double rightCommand = 0;
    double distanceAdjust = 0;
    double KpAim = -0.1;
    double minAimCommand = 0.05;
    double headingError = -x;
    double distanceError = -y;
    double steeringAdjust = 0.0;
    
    if(x > 1.0){
      steeringAdjust = KpAim * headingError - minAimCommand;
    }
    else if(x < -1.0){
    
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
    return RobotMap.frontLeftMotor.get() == 0 && 
    RobotMap.backLeftMotor.get() == 0 && 
    RobotMap.frontRightMotor.get() == 0 && 
    RobotMap.backRightMotor.get() == 0;
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
