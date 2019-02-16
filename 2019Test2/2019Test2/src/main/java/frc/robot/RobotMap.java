/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.*;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //Solenoid 
  public static Compressor c = new Compressor (0);
  public static DoubleSolenoid solenoidSteve = new DoubleSolenoid(3, 2);
  public static DoubleSolenoid solenoidStan = new DoubleSolenoid(1, 0); //TODO: change values
  public static Encoder armEncoder1 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  public static Encoder armEncoder2;
  
  public static final int[] canID = {0, 1, 2, 3}; 

  // This set for main robot. Be sure to comment out the practice robot definitions when running main bot.
  public static CANSparkMax frontLeftMotor  = new CANSparkMax(canID[1], MotorType.kBrushless);//check id-must set can id, change accordingly
  
  public static CANSparkMax backLeftMotor = new CANSparkMax(canID[0], MotorType.kBrushless);
  
  public static CANSparkMax frontRightMotor = new CANSparkMax(canID[2], MotorType.kBrushless);
  public static CANSparkMax backRightMotor = new CANSparkMax(canID[3], MotorType.kBrushless);
  public static CANEncoder encoder0 = RobotMap.frontRightMotor.getEncoder();
  public static CANEncoder encoder1 = RobotMap.frontLeftMotor.getEncoder();

  // This set for practice robot. Be sure to comment out the main robot definitions when running practice bot.
  /*
  public static CANSparkMax frontLeftMotor  = new CANSparkMax(canID[3], MotorType.kBrushless);//check id-must set can id, change accordingly
  public static CANSparkMax backLeftMotor = new CANSparkMax(canID[2], MotorType.kBrushless);
  public static CANSparkMax frontRightMotor = new CANSparkMax(canID[0], MotorType.kBrushless);
  public static CANSparkMax backRightMotor = new CANSparkMax(canID[1], MotorType.kBrushless);
  */  

  public static SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  public static SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  public static DifferentialDrive dDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  //speed controllergroup

  


  

  public static void init(){
   
    //encoder
    
    /*frontLeftMotor  = new CANSparkMax(canID[0], MotorType.kBrushless);//check id-must set can id, change accordingly
    backLeftMotor = new CANSparkMax(canID[1], MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(canID[2], MotorType.kBrushless);
    backRightMotor = new CANSparkMax(canID[3], MotorType.kBrushless);
    

    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);

    SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    dDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    */
    
  

   
  }




  


  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}