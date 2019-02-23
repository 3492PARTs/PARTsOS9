/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimeLight extends Subsystem {
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  double x,y,area,v;

  public LimeLight(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
  }

  @Override
  public void initDefaultCommand() {
  }

  public double getX(){
    x = tx.getDouble(0.0);
    return x;
  }
 
  public double getY(){
    y = ty.getDouble(0.0);
    return y;
  }

  public double getArea(){
    area = ta.getDouble(0.0);
    return area;
  }

  public double getV(){
    v = tv.getDouble(0.0);
    return v;
  }




}

