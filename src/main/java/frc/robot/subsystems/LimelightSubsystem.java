// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.APTree;
import frc.robot.utils.Pose;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-aptag");
  private final APTree tyToDistance = new APTree();
  private double distanceMeters = 0.0;

  // Example: fill with your measured mapping
  // ty (deg), distance (m)
  private static final double[][] TY_DISTANCE_TABLE = {
    {20.0, 0.864},
    {13, 1.27},
    {6.67, 1.86},
    {3.0, 2.26},
  };

  public double getDistanceMeters() {
    return distanceMeters;
  }

  public LimelightSubsystem() {
    tyToDistance.InsertValues(TY_DISTANCE_TABLE);
  }

  public double getDistanceMetersFromTy() {
    return tyToDistance.GetValue(getTY());
  }

  public double getDoubleEntry(String entry) {
    return limelight.getEntry(entry).getDouble(0);
  }

  public double[] getArraryEntry(String entry) {
    return limelight.getEntry(entry).getDoubleArray(new double[6]);
  }

  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1;
  }

  public double getTID() {
    return getDoubleEntry("tid");
  }

  public double getTA() {
    return getDoubleEntry("ta");
  }

  public double getTX() {
    return getDoubleEntry("tx");
  }

  public double getTY() {
    return getDoubleEntry("ty");
  }

  public boolean isTagAllowedForRotation() {
    int id = (int) Math.round(getTID());
    return id == 2 || id == 5 || id == 10;
  }

  public Pose getPoseFromTag(double robotYawDeg) {

    if (!hasValidTarget()) {
        return null; // or return current pose instead
    }

    double distance = distanceMeters;  // already updating in periodic
    double tx = getTX();

    // Bearing from field frame
    double bearingRad = Math.toRadians(robotYawDeg - tx);

    // Tag assumed at (0,0)
    double robotX = -distance * Math.cos(bearingRad);
    double robotY = -distance * Math.sin(bearingRad);

    return new Pose(robotX, robotY, robotYawDeg);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HAS TARGET", hasValidTarget());

    if (hasValidTarget()) {
      double ty = getTY();
      distanceMeters = tyToDistance.GetValue(ty);
      SmartDashboard.putNumber("tid", getTID());
      SmartDashboard.putNumber("ta", getTA());
      SmartDashboard.putNumber("ty", getTY());
      SmartDashboard.putNumber("tx", getTX());
      SmartDashboard.putNumber("Distance (m)", distanceMeters);
      //SmartDashboard.putNumber(key, value)
    }
  }

}
