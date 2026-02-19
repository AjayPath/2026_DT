// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

/**
 * Command to drive the robot to a specific field position and orientation.
 * 
 * Control Strategy:
 * - Translation: Independent X and Y PID controllers for precise positioning
 * - Rotation: Separate PID controller minimizes angle error
 * - All controllers run simultaneously for smooth, coordinated motion
 * 
 * Coordinate System:
 * - Field-relative control (x, y in meters, angle in degrees)
 * - X/Y controlled independently in field frame
 * - Shortest path rotation automatically calculated
 * 
 * Completion:
 * - Command finishes when both position and angle are within tolerance
 * - Odometry is updated to exact target pose on successful completion
 */
public class DriveToPoint extends Command {

  // ===========================================================================================
  // Dependencies
  // ===========================================================================================

  private final DriveSubsystem driveSubsystem;

  // ===========================================================================================
  // Control Parameters
  // ===========================================================================================

  private final APPID xPID;      // Controls X-axis translation
  private final APPID yPID;      // Controls Y-axis translation
  private final APPID turnPID;   // Controls rotation

  private final Pose targetPose;          // Desired final position and orientation
  private final double positionTolerance; // Acceptable position error (meters)
  private final double angleTolerance;    // Acceptable angle error (degrees)

  // ===========================================================================================
  // PID Constants - X Controller
  // ===========================================================================================

  private static final double kXP = 0.6;              // Proportional gain for X-axis
  private static final double kXI = 0.0;              // Integral gain for X-axis
  private static final double kXD = 0.05;             // Derivative gain for X-axis
  private static final double kXMaxSpeed = 0.25;     // Maximum X velocity (0-1 normalized)

  // ===========================================================================================
  // PID Constants - Y Controller
  // ===========================================================================================

  private static final double kYP = 0.6;              // Proportional gain for Y-axis
  private static final double kYI = 0.0;              // Integral gain for Y-axis
  private static final double kYD = 0.05;             // Derivative gain for Y-axis
  private static final double kYMaxSpeed = 0.25;     // Maximum Y velocity (0-1 normalized)

  // ===========================================================================================
  // PID Constants - Rotation
  // ===========================================================================================

  private static final double kTurnP = 0.02;              // Proportional gain for rotation
  private static final double kTurnI = 0.0;               // Integral gain for rotation
  private static final double kTurnD = 0.0;               // Derivative gain for rotation
  private static final double kMaxRotationSpeed = 0.25;   // Maximum rotation speed (0-1 normalized)



  // ===========================================================================================
  // Constructors
  // ===========================================================================================

  /**
   * Creates a command to drive to a specific field position and orientation.
   * 
   * @param driveSubsystem the drive subsystem
   * @param targetX target x position (meters)
   * @param targetY target y position (meters)
   * @param targetAngle target heading (degrees)
   * @param positionTolerance acceptable position error (meters)
   * @param angleTolerance acceptable angle error (degrees)
   */
  public DriveToPoint(DriveSubsystem driveSubsystem, double targetX, double targetY, 
                      double targetAngle, double positionTolerance, double angleTolerance) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = new Pose(targetX, targetY, targetAngle);
    this.positionTolerance = positionTolerance;
    this.angleTolerance = angleTolerance;

    // Initialize X-axis PID controller
    this.xPID = new APPID(kXP, kXI, kXD, positionTolerance);
    this.xPID.setMaxOutput(kXMaxSpeed);

    // Initialize Y-axis PID controller
    this.yPID = new APPID(kYP, kYI, kYD, positionTolerance);
    this.yPID.setMaxOutput(kYMaxSpeed);

    // Initialize rotation PID controller
    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleTolerance);
    this.turnPID.setMaxOutput(kMaxRotationSpeed);

    addRequirements(driveSubsystem);
  }

  /**
   * Creates a command with default tolerances (0.1m position, 2° angle).
   * 
   * @param driveSubsystem the drive subsystem
   * @param targetX target x position (meters)
   * @param targetY target y position (meters)
   * @param targetAngle target heading (degrees)
   */
  public DriveToPoint(DriveSubsystem driveSubsystem, double targetX, double targetY, double targetAngle) {
    this(driveSubsystem, targetX, targetY, targetAngle, 0.1, 2.0);
  }

  // ===========================================================================================
  // Command Lifecycle
  // ===========================================================================================

  @Override
  public void initialize() {
    // Reset all PID controllers
    xPID.reset();
    yPID.reset();
    turnPID.reset();

    // Log target for debugging
    // CHANGED: GetXValue() → getX(), GetYValue() → getY(), GetAngleValue() → getAngle()
    SmartDashboard.putNumber("TARGET_X", targetPose.getX());
    SmartDashboard.putNumber("TARGET_Y", targetPose.getY());
    SmartDashboard.putNumber("TARGET_ANGLE", targetPose.getAngle());

    System.out.println("DriveToPoint: Starting - Target: (" + 
                       targetPose.getX() + ", " + 
                       targetPose.getY() + ", " + 
                       targetPose.getAngle() + "°)");
  }

  @Override
  public void execute() {
    // Get current robot position
    // CHANGED: getCustomPose() → getPose() (matches cleaned DriveSubsystem)
    Pose currentPose = driveSubsystem.getPose();

    // ===========================================================================================
    // Translation Control - Independent X and Y
    // ===========================================================================================

    // Calculate position errors in field coordinates
    // CHANGED: GetXValue() → getX(), GetYValue() → getY()
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();

    // PID controllers drive each axis independently toward zero error
    xPID.setDesiredValue(0);
    yPID.setDesiredValue(0);

    // CHANGED: Removed negative sign (was double negative bug)
    // CHANGED: calcPID() → calculate() (matches cleaned APPID)
    // PID now receives -xError as input (negative because we want error to decrease toward 0)
    double xSpeed = xPID.calculate(-xError);
    double ySpeed = yPID.calculate(-yError);

    // REMOVED: Slew rate limiting - using PID output directly
    // Output velocities are already limited by PID maxOutput settings

    // Calculate distance for debugging
    double distanceToTarget = Math.sqrt(xError * xError + yError * yError);

    // ===========================================================================================
    // Rotation Control
    // ===========================================================================================

    // Normalize both angles to [0, 360) for consistent comparison
    // CHANGED: GetAngleValue() → getAngle()
    // CHANGED: NormalizeAngle360 → normalizeAngle360 (camelCase)
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle = Calculations.normalizeAngle360(targetPose.getAngle());

    // Calculate shortest angular path (handles wrapping, e.g., 350° to 10°)
    double angleError = Calculations.shortestAngularDistance(targetAngle, currentAngle);

    // PID drives angle error toward zero
    turnPID.setDesiredValue(0);
    // CHANGED: Removed negative sign and use -angleError (was double negative bug)
    // CHANGED: calcPID() → calculate()
    double rotationSpeed = turnPID.calculate(-angleError);

    // REMOVED: Slew rate limiting - using PID output directly

    // ===========================================================================================
    // Logging
    // ===========================================================================================

    SmartDashboard.putNumber("X_ERROR", xError);
    SmartDashboard.putNumber("Y_ERROR", yError);
    SmartDashboard.putNumber("DISTANCE_TO_TARGET", distanceToTarget);
    SmartDashboard.putNumber("ANGLE_ERROR", angleError);
    
    SmartDashboard.putNumber("X_VEL", xSpeed);
    SmartDashboard.putNumber("Y_VEL", ySpeed);
    SmartDashboard.putNumber("ROT_VEL", rotationSpeed);

    // ===========================================================================================
    // Drive Robot
    // ===========================================================================================

    // Apply both translation and rotation simultaneously
    // Field-relative mode (true) ensures x/y velocities are relative to field frame
    driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all robot motion
    driveSubsystem.drive(0.0, 0.0, 0.0, true);

    if (interrupted) {
      // CHANGED: getCustomPose() → getPose()
      // CHANGED: GetXValue() → getX(), GetYValue() → getY(), GetAngleValue() → getAngle()
      System.out.println("DriveToPoint: Command interrupted at (" + 
                         driveSubsystem.getPose().getX() + ", " + 
                         driveSubsystem.getPose().getY() + ", " + 
                         driveSubsystem.getPose().getAngle() + "°)");
    } else {
      // Command completed successfully - snap odometry to exact target
      // This prevents accumulated error from affecting subsequent paths
      // CHANGED: setOdom() → resetOdometry() (matches cleaned DriveSubsystem)
      // CHANGED: GetXValue() → getX(), GetYValue() → getY(), GetAngleValue() → getAngle()
      driveSubsystem.resetOdometry(new Pose(
          targetPose.getX(),
          targetPose.getY(),
          targetPose.getAngle()));

      System.out.println("DriveToPoint: Command completed successfully - Pose updated to target");
    }
  }

  @Override
  public boolean isFinished() {
    // CHANGED: getCustomPose() → getPose()
    Pose currentPose = driveSubsystem.getPose();

    // Calculate position error
    // CHANGED: GetXValue() → getX(), GetYValue() → getY()
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double distanceToTarget = Math.sqrt(xError * xError + yError * yError);
    boolean positionOnTarget = distanceToTarget <= positionTolerance;

    // Calculate angle error (using shortest path)
    // CHANGED: GetAngleValue() → getAngle()
    // CHANGED: NormalizeAngle360 → normalizeAngle360
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle = Calculations.normalizeAngle360(targetPose.getAngle());
    double angleError = Math.abs(Calculations.shortestAngularDistance(targetAngle, currentAngle));
    boolean angleOnTarget = angleError <= angleTolerance;

    // Command finishes only when both position AND angle are satisfied
    return positionOnTarget && angleOnTarget;
  }

  // ===========================================================================================
  // Status Methods
  // ===========================================================================================

  /**
   * Gets the current distance from robot to target position.
   * Useful for monitoring command progress.
   * 
   * @return distance to target in meters
   */
  public double getDistanceToTarget() {
    // CHANGED: getCustomPose() → getPose()
    // CHANGED: GetXValue() → getX(), GetYValue() → getY()
    Pose currentPose = driveSubsystem.getPose();
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    return Math.sqrt(xError * xError + yError * yError);
  }

  /**
   * Gets the current angle error (shortest path).
   * Useful for monitoring rotation progress.
   * 
   * @return angle error in degrees (positive = need to rotate CCW)
   */
  public double getAngleError() {
    // CHANGED: getCustomPose() → getPose()
    // CHANGED: GetAngleValue() → getAngle()
    // CHANGED: NormalizeAngle360 → normalizeAngle360
    Pose currentPose = driveSubsystem.getPose();
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle = Calculations.normalizeAngle360(targetPose.getAngle());
    return Calculations.shortestAngularDistance(targetAngle, currentAngle);
  }

  /**
   * Gets the target pose this command is driving to.
   * 
   * @return target pose
   */
  public Pose getTargetPose() {
    return new Pose(targetPose);
  }
}