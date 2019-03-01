/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCenterCmdGrp extends CommandGroup {
  
  public AutoCenterCmdGrp() {
    addSequential(new AutoDriveStraight(70, 0.6));
    addSequential(new AutoAim());
    addSequential(new AutoDriveStraight(5, 0.5));
    addSequential(new AutoDESTROY(1));
    //addSequential(new AutoAim());
    //addSequential(new AutoDriveStraight(15, 0.4)); // distance, direction
    //addSequential(new AutoDeployHatch());
    //addSequential(new AutoDriveStraight(15, -.5));
    addSequential(new DoNothing());
    addSequential(new DoNothing());
  }
}
