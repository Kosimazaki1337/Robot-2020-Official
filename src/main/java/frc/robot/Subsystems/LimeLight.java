/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  public final static int kTargetPipeline = 0;
  public final static int kBallPipeline = 1;

  private static final int verticalPixelsFOV = 320;
  private static final int horizontalPixelsFOV = 240;
  private static final double targetSizeMeters = 0.996;

  private static final double verticalViewDegrees = 59.6;
  private static final double horizontalViewDegrees = 49.7;

  NetworkTable mNetworkTable;
  NetworkTableEntry isTarget;
  NetworkTableEntry xOffset;
  NetworkTableEntry yOffset;

  public LimeLight(){
    mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

    isTarget = mNetworkTable.getEntry("tv");
    xOffset = mNetworkTable.getEntry("tx");
    yOffset = mNetworkTable.getEntry("ty");
  }

  @Override
  public void periodic() {
    update();
  }

  public double getDistance(){
    return (FOVinMeters()*verticalPixelsFOV)/(2*getTargetWidthInPixels()*
    Math.tan(Math.toRadians(getHalfFOVinDegrees())));
  }

  public synchronized double getXOffset() {
    double x = xOffset.getDouble(0);
    return x;
  }
  
  public double getTargetWidthInDegrees(){
    return verticalViewDegrees - 2*Math.abs(getXOffset());
  }

  public double getHalfFOVinDegrees(){
    return (verticalViewDegrees/2) + (getTargetWidthInDegrees()/2);
  }

  public double FOVinMeters(){
    return (verticalPixelsFOV*targetSizeMeters)/getTargetWidthInPixels();
  }

  public double getTargetWidthInPixels(){
    return (getTargetWidthInDegrees()/100)*verticalPixelsFOV;
  }

  public synchronized double getYOffset(){
    double y = yOffset.getDouble(0);
    return y;
  }

  public synchronized boolean isTargetVisible() {
    boolean isVisible = false;
    if(isTarget.getDouble(0) != 0){
      isVisible = true;
    }
    return isVisible;
  }

  public void logs(){
    SmartDashboard.putNumber("limelight_xOffset", getXOffset());
    SmartDashboard.putNumber("limelight_yOffset", getYOffset());
    SmartDashboard.putBoolean("limelight_isTarget", isTargetVisible());
    SmartDashboard.putNumber("limelight_DistanceToTarget", getDistance());
  }

  public void update(){
    
  }
}
