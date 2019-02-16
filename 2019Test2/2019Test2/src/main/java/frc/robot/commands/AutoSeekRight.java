package frc.robot.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoSeekRight extends Command {
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  double x,y,area,v;
  double KpDistance;

  public AutoSeekRight() {
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
    double headingError = -x;
    double distanceError = -y;
    double steeringAdjust = 0.0;
    double steering_adjust = 0;
    double kP2 = -0.03;
    double min_command = -0.03;

    if(v==0.0){
      //we don't see the target, seek for the target by spinning in place at a safe speed
      steering_adjust = 0.4;

    }
    else {
      //we do see the target, execute aiming code
       
       
      double heading_error = -x;
  
  
      if(x > 1.0){
        steering_adjust = kP2 * heading_error - min_command;
    
     
      }
      else if (x < -1.0){
        steering_adjust = kP2 * heading_error + min_command;
    
     
      }
      
    }
    leftCommand += steering_adjust;
    rightCommand -= steering_adjust;
    /*if (m_oi.driveStick.getRawAxis(4) < 0){
      RobotMap.dDrive.arcadeDrive(-0.3,rightCommand*.75);
    }
    if (m_oi.driveStick.getRawAxis(4) > 0){
      RobotMap.dDrive.arcadeDrive(-0.3,leftCommand*.75);
    }*/
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
