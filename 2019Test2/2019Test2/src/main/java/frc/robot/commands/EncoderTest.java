package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

/**
 * An example command.  You can replace me with your own command.
 */
public class EncoderTest extends Command {
    double startPositionRight;
    public double currentPositionRight;
    double pos;


  public EncoderTest() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_subsystem);
    
    
    
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startPositionRight = RobotMap.encoder0.getPosition();
  
    System.out.println("StartPOsitionRibht Just got INITIALIZED!");
 
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    //SmartDashboard.putNumber("start", startPositionRight);
    currentPositionRight = RobotMap.encoder0.getPosition() - startPositionRight;
    //SmartDashboard.putNumber("current", currentPositionRight);
   
  }

  public double sendCurrentPostion(){
      pos = currentPositionRight;
      //SmartDashboard.putNumber("AutoEncoderPosition", pos);
      return pos;
      
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
