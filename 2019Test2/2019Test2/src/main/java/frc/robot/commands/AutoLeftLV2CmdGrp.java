/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoLeftLV2CmdGrp extends CommandGroup {
  public AutoLeftLV2CmdGrp() {
    addSequential(new AutoDriveStraight(-81, 1));//encoders are inverted? so need to have negative value for fwd
    addSequential(new AutoSeekAimDestroy(-1, 5)); //seekDirection, destroyRotations
    addSequential(new AutoDriveStraight(20, 1));
    //addSequential(new AutoDeployHatch());
    addSequential(new AutoDriveStraight(10,-1));//distance, direction
    addSequential(new AutoTurn(-1,5)); //direction(assuming right is positive), rotations 
    addSequential(new AutoSeekAimDestroy(-1, 5)); //seekDirection, destroyRotations
    addSequential(new AutoDriveStraight(20, 1));
    addSequential(new AutoDriveStraight(10,-1));
    addSequential(new AutoTurn(-1, 5));
    addSequential(new AutoSeekAimDestroy(-1, 5));
    addSequential(new Lift(LiftLevel.Middle));
    addSequential(new AutoDriveStraight(10, 1));
   // addSequential(new AutoDeployHatch());
    addSequential(new AutoDriveStraight(10, -1));
    addSequential(new Lift(LiftLevel.Low));
    addSequential(new AutoTurn(1, 5));
  }
}
