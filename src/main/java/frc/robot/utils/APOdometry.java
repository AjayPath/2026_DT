// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;

/**
 * Field-oriented odometry for swerve drive.
 * Tracks each wheel independently, calculates robot center by averaging.
 * All positions/velocities are in field coordinates (meters).
 */
public class APOdometry {

  private static APOdometry instance;

  // Hardware
  private final Pigeon2 gyro;
  private final List<MAXSwerveModule> swerveMods;

  // Tracking state
  private final List<Pose> modulePoses;      // Each wheel's field position
  private final List<Vector> wheelOffsets;   // Fixed offsets from robot center
  private final double[] lastWheelDistances; // For delta calculation

  private Pose lastCenter;     // Last robot center pose
  private double lastTime;     // For velocity calculation
  private Vector lastVelocity; // Current velocity vector

  /**
   * Private constructor - use getInstance()
   * 
   * @param swerveMods list of modules in FL, FR, BL, BR order
   * @param gyro robot gyroscope
   */
  private APOdometry(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
    this.swerveMods = swerveMods;
    this.gyro = gyro;
    this.lastWheelDistances = new double[swerveMods.size()];
    this.modulePoses = new ArrayList<>(swerveMods.size());
    this.lastVelocity = new Vector(0, 0); // Initialize to zero vector

    // Calculate wheel offsets from robot center (meters)
    double halfWheelbase = DriveConstants.kWheelBase / 2.0;
    double halfTrackWidth = DriveConstants.kTrackWidth / 2.0;

    wheelOffsets = List.of(
        new Vector(halfWheelbase, halfTrackWidth),   // Front Left
        new Vector(halfWheelbase, -halfTrackWidth),  // Front Right
        new Vector(-halfWheelbase, halfTrackWidth),  // Back Left
        new Vector(-halfWheelbase, -halfTrackWidth)  // Back Right
    );

    // Initialize module poses at origin
    for (int i = 0; i < swerveMods.size(); i++) {
      double initialAngle = swerveMods.get(i).getPosition().angle.getDegrees();
      modulePoses.add(new Pose(0, 0, initialAngle));
    }

    lastCenter = new Pose(0, 0, gyro.getRotation2d().getDegrees());
    lastTime = Timer.getFPGATimestamp();
  }

  /**
   * Gets singleton instance.
   * 
   * @param swerveMods list of swerve modules
   * @param gyro pigeon2 gyroscope
   * @return APOdometry instance
   */
  public static APOdometry getInstance(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
    if (instance == null) {
      instance = new APOdometry(swerveMods, gyro);
    }
    return instance;
  }

  /**
   * Updates odometry from wheel movements.
   * Call this every robot loop (20ms recommended).
   * 
   * Process:
   * 1. Calculate each wheel's movement since last update
   * 2. Convert wheel movement to field coordinates using robot heading
   * 3. Update wheel positions in field frame
   * 4. Calculate robot center from wheel average
   * 5. Calculate velocity from position change
   */
  public void update() {
    double robotHeading = gyro.getRotation2d().getDegrees();
    double robotHeadingRad = Math.toRadians(robotHeading);
    
    // Update each wheel's field position
    for (int i = 0; i < swerveMods.size(); i++) {
      // Calculate distance traveled since last update
      double currentDistance = swerveMods.get(i).getPosition().distanceMeters;
      double deltaDistance = currentDistance - lastWheelDistances[i];
      lastWheelDistances[i] = currentDistance;

      // Get wheel angle (robot-relative)
      double wheelAngle = swerveMods.get(i).getPosition().angle.getDegrees();
      double wheelAngleRad = Math.toRadians(wheelAngle);

      // Calculate displacement in robot frame
      double dx_robot = Math.cos(wheelAngleRad) * deltaDistance;
      double dy_robot = Math.sin(wheelAngleRad) * deltaDistance;

      // Rotate to field frame using robot heading
      double dx_field = dx_robot * Math.cos(robotHeadingRad) - dy_robot * Math.sin(robotHeadingRad);
      double dy_field = dx_robot * Math.sin(robotHeadingRad) + dy_robot * Math.cos(robotHeadingRad);

      // Update wheel position in field coordinates
      modulePoses.get(i).setX(modulePoses.get(i).getX() + dx_field);
      modulePoses.get(i).setY(modulePoses.get(i).getY() + dy_field);
      modulePoses.get(i).setAngle(wheelAngle);
    }

    // Calculate robot center and velocity
    Pose currentCenter = calculateCenter();
    double currentTime = Timer.getFPGATimestamp();

    double dt = currentTime - lastTime;
    if (dt > 0) {
      // Velocity from position change (already in field frame)
      double dx = currentCenter.getX() - lastCenter.getX();
      double dy = currentCenter.getY() - lastCenter.getY();

      double mag = Math.sqrt(dx * dx + dy * dy) / dt;
      Rotation2d angle = new Rotation2d(Math.atan2(dy, dx));

      lastVelocity = new Vector(mag, angle);
    }

    lastCenter = currentCenter;
    lastTime = currentTime;
  }

  /**
   * Calculates robot center from one wheel's position.
   * 
   * @param wheelPose wheel's current pose
   * @param wheelOffset wheel's offset from center
   * @return calculated center pose
   */
  private Pose getCenterFromWheel(Pose wheelPose, Vector wheelOffset) {
    double yaw = gyro.getRotation2d().getDegrees();

    // Rotate offset by robot heading to get field-relative offset
    double offsetAngleRad = Math.toRadians(yaw + wheelOffset.getAngle().getDegrees());
    double offsetX = wheelOffset.getMagnitude() * Math.cos(offsetAngleRad);
    double offsetY = wheelOffset.getMagnitude() * Math.sin(offsetAngleRad);

    // Center = wheel position - offset
    return new Pose(
        wheelPose.getX() - offsetX,
        wheelPose.getY() - offsetY,
        yaw
    );
  }

  /**
   * Calculates robot center by averaging all wheel positions.
   * 
   * @return robot center pose in field coordinates
   */
  private Pose calculateCenter() {
    double sumX = 0;
    double sumY = 0;

    // Get center estimate from each wheel and average
    for (int i = 0; i < modulePoses.size(); i++) {
      Pose centerFromThisWheel = getCenterFromWheel(modulePoses.get(i), wheelOffsets.get(i));
      sumX += centerFromThisWheel.getX();
      sumY += centerFromThisWheel.getY();
    }

    double avgX = sumX / modulePoses.size();
    double avgY = sumY / modulePoses.size();
    double avgAngle = gyro.getRotation2d().getDegrees();

    return new Pose(avgX, avgY, avgAngle);
  }

  /**
   * Gets robot pose with continuous angle (can exceed 360°).
   * Use for control calculations.
   * 
   * @return copy of pose with continuous angle
   */
  public Pose getPoseContinuous() {
    return new Pose(lastCenter);
  }

  /**
   * Gets robot pose with normalized angle [0, 360).
   * Use for display and logging.
   * 
   * @return copy of pose with normalized angle
   */
  public Pose getPose() {
    Pose normalizedPose = new Pose(lastCenter);
    normalizedPose.setAngle(Calculations.normalizeAngle360(lastCenter.getAngle()));
    return normalizedPose;
  }

  /**
   * Gets current velocity vector.
   * 
   * @return velocity (magnitude in m/s, direction in degrees)
   */
  public Vector getVelocity() {
    return lastVelocity;
  }

  /**
   * Gets all module poses.
   * 
   * @return copy of list of module poses
   */
  public List<Pose> getModulePoses() {
    return new ArrayList<>(modulePoses);
  }

  /**
   * Gets specific module pose.
   * 
   * @param index module index (0-3)
   * @return copy of module pose
   */
  public Pose getModulePose(int index) {
    return new Pose(modulePoses.get(index));
  }

  /**
   * Sets robot pose to specific position.
   * Updates all wheel positions to match.
   * 
   * @param newPose desired robot pose
   */
  public void setPose(Pose newPose) {
    double yaw = newPose.getAngle();
    
    for (int i = 0; i < swerveMods.size(); i++) {
      Vector offset = wheelOffsets.get(i);
      
      // Rotate offset to field frame
      double offsetAngleRad = Math.toRadians(yaw + offset.getAngle().getDegrees());
      double offsetX = offset.getMagnitude() * Math.cos(offsetAngleRad);
      double offsetY = offset.getMagnitude() * Math.sin(offsetAngleRad);
      
      double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();
      
      modulePoses.get(i).setPose(
          newPose.getX() + offsetX,
          newPose.getY() + offsetY,
          steerAngle
      );
      
      lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;
    }
    
    lastCenter = new Pose(newPose);
    calculateCenter();
  }

  /**
   * Resets odometry to origin with current gyro heading.
   */
  public void reset() {
    double yaw = gyro.getRotation2d().getDegrees();
    
    for (int i = 0; i < swerveMods.size(); i++) {
      Vector offset = wheelOffsets.get(i);
      
      // Rotate offset to field frame
      double offsetAngleRad = Math.toRadians(yaw + offset.getAngle().getDegrees());
      double offsetX = offset.getMagnitude() * Math.cos(offsetAngleRad);
      double offsetY = offset.getMagnitude() * Math.sin(offsetAngleRad);
      
      double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();
      
      modulePoses.get(i).setPose(offsetX, offsetY, steerAngle);
      lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;
    }
    
    lastCenter = new Pose(0, 0, yaw);
  }

  /**
   * Publishes odometry data to SmartDashboard.
   */
  public void publishToSmartDashboard() {
    Pose pose = getPose();
    
    // Robot pose
    SmartDashboard.putNumber("Odometry/X", pose.getX());
    SmartDashboard.putNumber("Odometry/Y", pose.getY());
    SmartDashboard.putNumber("Odometry/Angle", pose.getAngle());
    
    // Velocity
    SmartDashboard.putNumber("Odometry/Velocity Magnitude", lastVelocity.getMagnitude());
    SmartDashboard.putNumber("Odometry/Velocity Angle", lastVelocity.getAngle().getDegrees());
    
    // Individual wheel poses (optional - can be verbose)
    for (int i = 0; i < modulePoses.size(); i++) {
      Pose wheelPose = modulePoses.get(i);
      SmartDashboard.putNumber("Odometry/Wheel " + i + " X", wheelPose.getX());
      SmartDashboard.putNumber("Odometry/Wheel " + i + " Y", wheelPose.getY());
      SmartDashboard.putNumber("Odometry/Wheel " + i + " Angle", wheelPose.getAngle());
    }
  }

  /**
   * Logs wheel poses to console (for debugging).
   */
  public void logWheelPoses() {
    for (int i = 0; i < swerveMods.size(); i++) {
      modulePoses.get(i).print("Wheel", i);
    }
  }
}