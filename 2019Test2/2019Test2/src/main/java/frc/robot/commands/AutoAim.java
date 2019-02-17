/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.*;
public class AutoAim extends Command {
  LimeLight limeLight = new LimeLight();
  double x,y,area,v;
  double KpDistance;

  public AutoAim() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    x = limeLight.getX();
    y = limeLight.getY();
    area = limeLight.getArea();
    v = limeLight.getV();

    KpDistance = -0.1;
    double leftCommand = 0;
    double rightCommand = 0;
    double steering_adjust = 0;
    double min_command = -0.03;
    double heading_error = -x;
    double Kp = -0.06;

    if(x > 1.0){
      steering_adjust = Kp * heading_error - min_command;
      
       
    }
    else if (x < 1.0){
      steering_adjust = Kp * heading_error + min_command;
      
       
    }
    leftCommand += steering_adjust;
    rightCommand -= steering_adjust;
    RobotMap.dDrive.tankDrive(leftCommand*.75,rightCommand*.75);     
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return x > -1 && x < 1;
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