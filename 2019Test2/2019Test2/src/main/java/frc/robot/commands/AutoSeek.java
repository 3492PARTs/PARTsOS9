package frc.robot.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.*;

public class AutoSeek extends Command {
  private double KpDistance;
  private LimeLight limeLight;
  private double direction;

  public AutoSeek(double direction) {
    limeLight = new LimeLight();
    this.direction = direction;
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
    KpDistance = -0.1;
    double leftCommand = 0;
    double rightCommand = 0;
    double steering_adjust = 0;
    double kP2 = -0.03;
    double min_command = -0.03;

    if(limeLight.getV()==0.0){
      //we don't see the target, seek for the target by spinning in place at a safe speed
      steering_adjust = 0.4;

    }
    else {
      //we do see the target, execute aiming code
       
       
      double heading_error = -limeLight.getX();
  
  
      if(limeLight.getX() > 1.0){
        steering_adjust = kP2 * heading_error - min_command;
    
     
      }
      else if (limeLight.getX() < -1.0){
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
    RobotMap.dDrive.tankDrive(direction*leftCommand*.75,direction*rightCommand*.75);  
    
  
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
