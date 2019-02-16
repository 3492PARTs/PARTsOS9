/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Encoder extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
    private static double startingPosition0 = 0;
    private static double startingPosition1 = 0;


  @Override
  public void initDefaultCommand() {

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getEncoderValue0(){
    return RobotMap.encoder0.getPosition() - startingPosition0;
  }

  public double getEncoderValue1(){
    return RobotMap.encoder1.getPosition() - startingPosition1;
  }

  public double getEncoder0Distance(){      // in inches
    if (RobotMap.solenoidStan.get() == DoubleSolenoid.Value.kForward){
      return Math.abs(rotationsToInchesLow(RobotMap.encoder0.getPosition() - startingPosition0));
    }
    else if (RobotMap.solenoidStan.get() == DoubleSolenoid.Value.kReverse){
      return Math.abs(rotationsToInchesHigh(RobotMap.encoder0.getPosition() - startingPosition0));
    }
    else{
      return -1; //means its off
    }
  }

  public double getEncoder1Distance(){      // in inches
    if (RobotMap.solenoidStan.get() == DoubleSolenoid.Value.kForward){
      return Math.abs(rotationsToInchesLow(RobotMap.encoder1.getPosition() - startingPosition1));
    }
    else if (RobotMap.solenoidStan.get() == DoubleSolenoid.Value.kReverse){
      return Math.abs(rotationsToInchesHigh(RobotMap.encoder1.getPosition() - startingPosition1));
    }
    else{
      return -1; //means its off
    }
  }

  public void reset1(){
    startingPosition1 = RobotMap.encoder1.getPosition();
  }

  public void reset0(){
    startingPosition0 = RobotMap.encoder0.getPosition();
  }

  public void resetAll(){
    reset0();
    reset1();
  }

  private double rotationsToInchesLow(double rotations){
    return rotations*((Math.PI * 6.25) / ((42/12) * (60/14))); //to find inches covered by rotation of motor in low gear
  }
  private double rotationsToInchesHigh(double rotations){
    return rotations*((Math.PI * 6.25) / ((42/12) * (40/34))); //to find inches covered by rotation of motor in high gear
  }
}

