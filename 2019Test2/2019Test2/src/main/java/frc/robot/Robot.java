
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.commands.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  double setPositionRight;
  LimeLight limeLight = new LimeLight();
  Encoder encoder = new Encoder();
  double KpDistance = -0.1;
  boolean firstPress8 = true;
  double startPosition;

  @Override
  public void robotInit() {
    RobotMap.liftMotor2.set(ControlMode.Follower, 14);
    RobotMap.intake2.set(ControlMode.Follower, 11);
    m_oi = new OI();
    SmartDashboard.putData("Auto mode", m_chooser);
    m_chooser.setDefaultOption("Default Auto", new AutoDriveFwdCmdGrp());
    m_chooser.addOption("RightRocketLV2Auto", new AutoRightLV2CmdGrp());
    m_chooser.addOption("LeftRocketLV2Auto", new AutoLeftLV2CmdGrp());
    m_chooser.addOption("CenterCargoAuto", new AutoCenterCmdGrp());
    m_chooser.addOption("DoNothingAuto", new AutoDoNothingCmdGrp());    
    RobotMap.c.setClosedLoopControl(true);
    RobotMap.solenoidStan.set(DoubleSolenoid.Value.kForward); //TODO: check if forward is low gear
  }
    
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("DriveDistanceAuto", encoder.getEncoder0Distance());
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   
    startPosition = RobotMap.liftMotor.getSelectedSensorPosition();
  }
  ArmPosition armPosition = ArmPosition.home;
  @Override
  public void teleopPeriodic() {
    /*
    if (RobotMap.liftEncoder.getDistance() >= 1 && RobotMap.liftEncoder.getDistance() <= 1 && armPosition == ArmPosition.home){
      new ArmPivot(ArmPosition.miss).start();
      armPosition = ArmPosition.miss;
    }
    else if (RobotMap.liftEncoder.getDistance() <= 1 && RobotMap.liftEncoder.getDistance() >= 1 && armPosition == ArmPosition.miss){
      new ArmPivot(ArmPosition.home).start();
      armPosition = ArmPosition.home;
    }
    */
    Scheduler.getInstance().run();
    double x = limeLight.getX();
    double y = limeLight.getY();
    double area = limeLight.getArea();
    double v = limeLight.getV();
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Target Valid", v);
    SmartDashboard.putNumber("Drive Distance", encoder.getEncoder0Distance());
    SmartDashboard.putNumber("Lift Encoder Distance", RobotMap.liftMotor.getSelectedSensorPosition());

    RobotMap.dDrive.arcadeDrive(-m_oi.driveStick.getRawAxis(1)*.8,m_oi.driveStick.getRawAxis(4)*.5);
   
    double Kp = -0.06;
    double min_command = -0.03;
    double leftCommand = 0;
    double rightCommand = 0;
    double steering_adjust = 0;double startPos;
    double distanceAdjust = 0;
   /* 
    //AIMING AIMING AIMING AIMING
   if(m_oi.driveStick.getRawButton(4)){  //Y
      double heading_error = -x;
      if(x > 1.0){
        steering_adjust = Kp * heading_error - min_command; 
      }
      else if (x < -1.0){
        steering_adjust = Kp * heading_error + min_command; 
      }
      leftCommand += steering_adjust;
      rightCommand -= steering_adjust;
      RobotMap.dDrive.tankDrive(leftCommand*.75,rightCommand*.75);      
    } //END AIMING 

    //SEEKING SEEKING SEEKING SEEKING
    double kP2 = -0.03;
    if(m_oi.driveStick.getRawButton(3)){        
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
        //if (m_oi.driveStick.getRawAxis(4) < 0){
         // RobotMap.dDrive.arcadeDrive(-0.3,rightCommand*.75);
        //}
        //if (m_oi.driveStick.getRawAxis(4) > 0){
         // RobotMap.dDrive.arcadeDrive(-0.3,leftCommand*.75);
        //}
        RobotMap.dDrive.tankDrive(leftCommand*.75,rightCommand*.75);  
    } 
    //END SEEKING
    
    double KpAim = -0.1;
    double minAimCommand = 0.05;
    if (m_oi.driveStick.getRawButton(5))
    {
      double headingError = -x;
      double distanceError = -y;
      double steeringAdjust = 0.0;      
      if(x > 1.0){
        steeringAdjust = KpAim * headingError - minAimCommand;
      }
      else if(x < -1.0){
      steeringAdjust = KpAim*headingError+minAimCommand; 
      }
      distanceAdjust = KpDistance * distanceError;
      leftCommand += steeringAdjust + distanceAdjust;
      rightCommand -= steeringAdjust + distanceAdjust;
      //The multiplier for the speed here does NOT make the actual speed go down by that much.
      RobotMap.dDrive.tankDrive((leftCommand)*.3,(-rightCommand)*.3);
    } */
  // end aim, seek, destroy
    if(m_oi.driveStick.getRawButton(1)){
      RobotMap.solenoidSteve.set(DoubleSolenoid.Value.kForward);
    } 

    if(m_oi.driveStick.getRawButton(2)){
      RobotMap.solenoidSteve.set(DoubleSolenoid.Value.kReverse);
    }  
    
    if(m_oi.driveStick.getRawButton(3)){
      RobotMap.armMotor.set(ControlMode.PercentOutput, 1);
    }
    else if(m_oi.driveStick.getRawButton(4)){
      RobotMap.armMotor.set(ControlMode.PercentOutput, -1);
    }
    else{
      RobotMap.armMotor.set(ControlMode.PercentOutput, 0);
    }

    if(m_oi.driveStick.getRawButton(5)){
      RobotMap.intake.set(ControlMode.PercentOutput, 1);
    }
    else if(m_oi.driveStick.getRawButton(6)){
      RobotMap.intake.set(ControlMode.PercentOutput, -.775);
    }
    else{
      RobotMap.intake.set(ControlMode.PercentOutput, 0);
    }
    if(m_oi.launchPad.getRawButton(1)){
      RobotMap.liftMotor.set(ControlMode.PercentOutput, 1);
    }
    else if(m_oi.launchPad.getRawButton(3)){
      RobotMap.liftMotor.set(ControlMode.PercentOutput, -1);
    }
    else{
      RobotMap.liftMotor.set(ControlMode.PercentOutput, 0);
    }
    
    if (m_oi.launchPad.getRawButton(8)){
      if((RobotMap.liftMotor.getSelectedSensorPosition() - startPosition) < -34206){
        RobotMap.liftMotor.set(ControlMode.PercentOutput, 1);
      } else if(RobotMap.liftMotor.getSelectedSensorPosition() - startPosition > -34206){
        RobotMap.liftMotor.set(ControlMode.PercentOutput, -1);
      }
      else RobotMap.liftMotor.set(ControlMode.PercentOutput, 0);
    }
    
    if (m_oi.launchPad.getRawButton(9)){
      if((RobotMap.liftMotor.getSelectedSensorPosition() - startPosition) < -70270){
        RobotMap.liftMotor.set(ControlMode.PercentOutput, 1);
      }
      else if((RobotMap.liftMotor.getSelectedSensorPosition() - startPosition) > -70270){
        RobotMap.liftMotor.set(ControlMode.PercentOutput, -1);
      }
      else{
        RobotMap.liftMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    if(m_oi.launchPad.getRawButton(5)){
      if(RobotMap.liftMotor.getSelectedSensorPosition() > startPosition){
        RobotMap.liftMotor.set(ControlMode.PercentOutput, 0);
      }
      else if (RobotMap.liftMotor.getSelectedSensorPosition() < startPosition){
        RobotMap.liftMotor.set(ControlMode.PercentOutput, 1);
      }
    }
  }

  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}