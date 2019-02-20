
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
//import edu.wpi.first.wpilibj.Encoder;
import frc.robot.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
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

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
     
    //network table
     
    RobotMap.liftMotor2.set(ControlMode.Follower, 14);
    RobotMap.intake2.set(ControlMode.Follower, 11);
    
    
    //RobotMap.liftEncoder.setDistancePerPulse(1.3 * Math.PI);
    //RobotMap.armEncoder.setDistancePerPulse(100 * (35/12));
    m_oi = new OI();
    SmartDashboard.putData("Auto mode", m_chooser);
    m_chooser.setDefaultOption("Default Auto", new AutoDriveFwdCmdGrp());
    m_chooser.addOption("RightRocketLV2Auto", new AutoRightLV2CmdGrp());
    m_chooser.addOption("LeftRocketLV2Auto", new AutoLeftLV2CmdGrp());
    m_chooser.addOption("CenterCargoAuto", new AutoCenterCmdGrp());
    m_chooser.addOption("DoNothingAuto", new AutoDoNothingCmdGrp());    
    RobotMap.c.setClosedLoopControl(true);
    //RobotMap.frontLeftMotor.setInverted(true);
    //RobotMap.backLeftMotor.setInverted(true);
    RobotMap.solenoidStan.set(DoubleSolenoid.Value.kForward); //TODO: check if forward is low gear

    
  }
    //startPosition = RobotMap.encoder0.getPosition();
    
   /* 
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*double x = tx.getDouble(0.0);     //horizontal distance from target
    double y = ty.getDouble(0.0);     //vertical distance from target
    double area = ta.getDouble(0.0);  //target area on screen--the larger the area, the closer the target
    double v = tv.getDouble(0.0);     //a number that tells if the target is valid            
    */

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    
    
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

  
    
    SmartDashboard.putNumber("DriveDistanceAuto", encoder.getEncoder0Distance());
    /*if(encoderTest.currentPositionRight <= setPositionRight){
      RobotMap.dDrive.arcadeDrive(-.25, 0);
      System.out.println("IM SUPPOSED TO BE DRIVING");
    }
    else {
     RobotMap.dDrive.arcadeDrive(0, 0);
     System.out.println("IM IN ELSE");
    }*/

    


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   
    
    startPosition = RobotMap.liftMotor.getSelectedSensorPosition();

    

    
  }

  ArmPosition armPosition = ArmPosition.home;
  /**
   * This function is called periodically during operator control.
   */
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
    //SmartDashboard.putNumber("ArmEncoder", RobotMap.armEncoder.getDistance());
   // SmartDashboard.putNumber("LiftEncoder", RobotMap.liftEncoder.getDistancePerPulse());
    //System.out.println("start: " + startPosition);
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
   /* SmartDashboard.putNumber("Encoder Position", -RobotMap.encoder0.getPosition());
    System.out.println("Encoder Position: " + -RobotMap.encoder0.getPosition());
    SmartDashboard.putNumber("velocity", RobotMap.encoder0.getVelocity());*/
    //send info to the console as to whether the compressor thinks it is on or off
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