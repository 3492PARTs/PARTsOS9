
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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


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

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  
  double KpDistance = -0.1;
   

  public double EstimateDistance() {
    double distance;
    double y = ty.getDouble(0.0);
    SmartDashboard.putNumber("Angle of vertical offset is " , y);
    System.out.println("Angle of vertical offset is " + y);
   
    double rocketBallTargetHeight = 37;
    distance = ((rocketBallTargetHeight - 9.5)/Math.abs(Math.tan(Math.toRadians(y))));
    return distance;
  }
  //double startPosition;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
     
    //network table
    RobotMap.liftMotor2.set(ControlMode.Follower, 4);
    RobotMap.intake2.set(ControlMode.Follower, 7);
    RobotMap.liftEncoder.setDistancePerPulse(1.3 * Math.PI);
    RobotMap.armEncoder.setDistancePerPulse(100 * (35/12));
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new AutoDriveStraight(1, 1));
    //m_chooser.addOption("Right, Level 2", new AutoRightL2());
    SmartDashboard.putData("Auto mode", m_chooser);
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
  
    System.out.println("IM IN AUTO");
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
    


    

    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //System.out.println("start: " + startPosition);
    Scheduler.getInstance().run();

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Target Valid", v);
    
    SmartDashboard.putNumber("Encoder Position", -RobotMap.encoder0.getPosition());
    System.out.println("Encoder Position: " + -RobotMap.encoder0.getPosition());
    SmartDashboard.putNumber("velocity", RobotMap.encoder0.getVelocity());
    //System.out.println("vEncoder Position: " + RobotMap.encoder0.getVelocity());

   /* SmartDashboard.putNumber("3Encoder Position", -1*(RobotMap.encoder3.getPosition()));
    System.out.println("3Encoder Position: " + RobotMap.encoder3.getPosition());
    SmartDashboard.putNumber("3vEncoder Position", -1*(RobotMap.encoder3.getVelocity()));
    System.out.println("3vEncoder Position: " + RobotMap.encoder3.getVelocity());*/

    //send info to the console as to whether the compressor thinks it is on or off
    System.out.println("The compressor thinks it's on:  " + RobotMap.c.enabled()); 
    System.out.println("The pressure switch value is: " + RobotMap.c.getPressureSwitchValue());
    System.out.println("The compressor current is: " + RobotMap.c.getCompressorCurrent());

    RobotMap.dDrive.arcadeDrive(-m_oi.driveStick.getRawAxis(1)*.8,m_oi.driveStick.getRawAxis(4)*.5);
    

    double Kp = -0.06;
    double min_command = -0.03;
    double leftCommand = 0;
    double rightCommand = 0;
    double steering_adjust = 0;
    double distanceAdjust = 0;
  

    //RobotMap.dDrive.tankDrive(-m_oi.driveStick.getRawAxis(1)*0.5+leftCommand, -m_oi.driveStick.getRawAxis(5)*0.5+rightCommand);

    //AIMING AIMING AIMING AIMING
   /* if(m_oi.driveStick.getRawButton(4)){  //Y
      double heading_error = -x;
      
      
      if(x > 1.0){
        steering_adjust = Kp * heading_error - min_command;
        
         
      }
      else if (x < 1.0){
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
        /*if (m_oi.driveStick.getRawAxis(4) < 0){
          RobotMap.dDrive.arcadeDrive(-0.3,rightCommand*.75);
        }
        if (m_oi.driveStick.getRawAxis(4) > 0){
          RobotMap.dDrive.arcadeDrive(-0.3,leftCommand*.75);
        }
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
  
      RobotMap.dDrive.tankDrive((-leftCommand)*.3,(rightCommand)*.3);
    }
    */
    


    if(m_oi.driveStick.getRawButton(1)){
      RobotMap.solenoidSteve.set(DoubleSolenoid.Value.kForward);
    } 

    if(m_oi.driveStick.getRawButton(2)){
      RobotMap.solenoidSteve.set(DoubleSolenoid.Value.kReverse);
    }  

    if(m_oi.driveStick.getRawButton(3)){
      RobotMap.armMotor.set(ControlMode.PercentOutput, 0.25);
    }

    if(m_oi.driveStick.getRawButton(4)){
      RobotMap.armMotor.set(ControlMode.PercentOutput, -0.25);
    }

    if(m_oi.driveStick.getRawButton(5)){
      RobotMap.intake.set(ControlMode.PercentOutput, 0.5);
    }

    if(m_oi.driveStick.getRawButton(6)){
      RobotMap.intake.set(ControlMode.PercentOutput, -0.5);
    }

    if(m_oi.controlStick.getRawButton(3)){
      RobotMap.liftMotor.set(ControlMode.PercentOutput, -0.5);
    }

    if(m_oi.controlStick.getRawButton(4)){
      RobotMap.liftMotor.set(ControlMode.PercentOutput, 0.5);
    }
    
    if(m_oi.launchPad.getRawButton(1)){
      System.out.println("b1 works");
    }

    if(m_oi.launchPad.getRawButton(3)){
      System.out.println("b2 works");
    }

    if(m_oi.launchPad.getRawButton(5)){
      System.out.println("b3 works");
    }

    if(m_oi.launchPad.getRawButton(8)){
      System.out.println("b4 works");
    }

    if(m_oi.launchPad.getRawButton(9)){
      System.out.println("b5 works");
    }

     





  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}