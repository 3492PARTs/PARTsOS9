/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.*;

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

  public void reset0(){
      startingPosition0 = RobotMap.encoder0.getPosition();
  }

  public double getEncoderValue0(){
    return RobotMap.encoder0.getPosition() - startingPosition0;
  }

  public void reset1(){
    startingPosition1 = RobotMap.encoder1.getPosition();
}

public double getEncoderValue1(){
  return RobotMap.encoder1.getPosition() - startingPosition1;
}

public void resetAll(){
  reset0();
  reset1();
}
}

