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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;

public class RobotMap {
  //pneumatics 
  public static Compressor c = new Compressor (0);
  public static DoubleSolenoid solenoidStan = new DoubleSolenoid(1, 0); //TODO: change values and this is gear shifter solenoid
  public static Solenoid solenoidSteve = new Solenoid(4);
  //public static Solenoid solenoidSarah = new Solenoid(3 or 2);
  public static final int[] canID = {0, 1, 2, 3};
  // This set for main robot. Be sure to comment out the practice robot definitions when running main bot.
  public static CANSparkMax frontLeftMotor  = new CANSparkMax(canID[1], MotorType.kBrushless);
  public static CANSparkMax backLeftMotor = new CANSparkMax(canID[0], MotorType.kBrushless);
  public static CANSparkMax frontRightMotor = new CANSparkMax(canID[2], MotorType.kBrushless);
  public static CANSparkMax backRightMotor = new CANSparkMax(canID[3], MotorType.kBrushless);
  public static CANEncoder encoder0 = backRightMotor.getEncoder();
  public static CANEncoder encoder1 = backLeftMotor.getEncoder();
  public static SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  public static SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  public static DifferentialDrive dDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
  //Lift motor - talon
  public static TalonSRX liftMotor = new TalonSRX(14);
  public static TalonSRX liftMotor2 = new TalonSRX(15);
  public static TalonSRX armMotor = new TalonSRX(5);
  public static TalonSRX intake = new TalonSRX(11);
 
 
  // This set for practice robot. Be sure to comment out the main robot definitions when running practice bot.
  /*
  public static CANSparkMax frontLeftMotor  = new CANSparkMax(canID[3], MotorType.kBrushless);//check id-must set can id, change accordingly
  public static CANSparkMax backLeftMotor = new CANSparkMax(canID[2], MotorType.kBrushless);
  public static CANSparkMax frontRightMotor = new CANSparkMax(canID[0], MotorType.kBrushless);
  public static CANSparkMax backRightMotor = new CANSparkMax(canID[1], MotorType.kBrushless);
  */  

  public static void init(){
  }
}