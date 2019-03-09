/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class GearShift extends Command {
    private OI m_oi;
    private Boolean highGear = false;
    private Boolean TimesUp = false;
    long startTime;
    TwoSecTimer twoSecTimer = new TwoSecTimer();
  public GearShift() {
      m_oi = new OI();
    requires(Robot.m_subsystem);
  }


  @Override
  protected void initialize() {
  }
  
  private long stTime = 0; 
  private boolean hold = true;

  @Override
  protected void execute() {
    RobotMap.solenoidStan.set(DoubleSolenoid.Value.kForward);
    /*if (Math.abs(RobotMap.encoder0.getVelocity()) >= 3200){
      if (hold){
        stTime = System.currentTimeMillis();
        hold = false;
      }
      if (System.currentTimeMillis() >= stTime + 2000){
        RobotMap.solenoidStan.set(DoubleSolenoid.Value.kForward);
        highGear = true;
        SmartDashboard.putBoolean("Gear shifted to high ", highGear);
      }
    }
    else if (Math.abs(m_oi.driveStick.getRawAxis(1)) < 0.01){
      RobotMap.solenoidStan.set(DoubleSolenoid.Value.kReverse);
      highGear = false;
      SmartDashboard.putBoolean("Gear shifted to high ", highGear);
      hold = true;
    }
    else if (Math.abs(RobotMap.encoder0.getVelocity()) < 3200) hold = true;*/
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
