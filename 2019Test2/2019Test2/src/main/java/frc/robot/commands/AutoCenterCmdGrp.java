/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCenterCmdGrp extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoCenterCmdGrp() {
    
    addSequential(new AutoDriveStraight(70, 0.5));
    
    addSequential(new AutoAim());
    addSequential(new AutoDriveStraight(5, 0.5));
    addSequential(new AutoDESTROY(1));
    addSequential(new AutoAim());
    addSequential(new AutoDriveStraight(15, 0.4)); // distance, direction
    addSequential(new AutoDeployHatch());
    addSequential(new AutoDriveStraight(20, -.5));
    addSequential(new DoNothing());
    addSequential(new DoNothing());

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
