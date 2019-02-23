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
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() { 
    System.out.println("auto seek");
    KpDistance = -0.1;
    double leftCommand = 0;
    double rightCommand = 0;
    double steering_adjust = 0;
    double kP2 = -0.03;
    double min_command = -0.03;

    if(limeLight.getV()==0.0){
      steering_adjust = 0.4;
    }
    else {
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
    RobotMap.dDrive.tankDrive(direction*leftCommand*.75,direction*rightCommand*.75);  
} 
  @Override
  protected boolean isFinished() {
    return limeLight.getX() > -1 && limeLight.getX() < 1;
  }

  @Override
  protected void end() {
    RobotMap.dDrive.arcadeDrive(0, 0);
    System.out.println("auto seek finish");
  }

  @Override
  protected void interrupted() {
  }
}
