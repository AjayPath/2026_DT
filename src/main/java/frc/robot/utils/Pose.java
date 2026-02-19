// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Represents a 2D pose with x, y position and heading angle.
 * Stores raw angle values without automatic wrapping for consistent odometry tracking.
 * All distance measurements are in meters, angles in degrees.
 */
public class Pose {
  
  private double x;
  private double y;
  private double angle;

  // ===========================================================================================
  // Constructors
  // ===========================================================================================

  /**
   * Creates a pose with specified position and angle.
   * 
   * @param x x component in meters
   * @param y y component in meters
   * @param angle heading angle in degrees
   */
  public Pose(double x, double y, double angle) {
    this.x = x;
    this.y = y;
    this.angle = angle;
  }

  /**
   * Creates a pose at origin with 0 degree heading.
   */
  public Pose() {
    this.x = 0;
    this.y = 0;
    this.angle = 0;
  }

  /**
   * Copy constructor - creates a new pose from an existing pose.
   * 
   * @param other the pose to copy
   */
  public Pose(Pose other) {
    this.x = other.x;
    this.y = other.y;
    this.angle = other.angle;
  }

  // ===========================================================================================
  // Pose Operations
  // ===========================================================================================

  /**
   * Reflects the pose across the y-axis.
   */
  public void reflectY() {
    this.x = -this.x;
    this.angle = 180 - this.angle;
    this.angle = Calculations.normalizeAngle(this.angle);
  }

  /**
   * Adds a vector to the pose position.
   * 
   * @param v the vector to add
   */
  public void addVector(Vector v) {
    x += v.getX();
    y += v.getY();
  }

  /**
   * Subtracts a vector from the pose position.
   * 
   * @param v the vector to subtract
   */
  public void subtractVector(Vector v) {
    x -= v.getX();
    y -= v.getY();
  }

  /**
   * Calculates the displacement vector from another pose to this pose.
   * 
   * @param p the pose to subtract from this pose
   * @return displacement vector
   */
  public Vector subtract(Pose p) {
    return new Vector(x - p.getX(), y - p.getY());
  }

  /**
   * Transforms the pose heading by adding a gyro angle.
   * Note: This only modifies the heading angle, not the position.
   * 
   * @param gyroAngle the robot's heading in degrees to add
   */
  public void transform(double gyroAngle) {
    angle = angle + gyroAngle;
  }

  // ===========================================================================================
  // Getters
  // ===========================================================================================

  /**
   * Gets the x component.
   * 
   * @return x component in meters
   */
  public double getX() {
    return x;
  }

  /**
   * Gets the y component.
   * 
   * @return y component in meters
   */
  public double getY() {
    return y;
  }

  /**
   * Gets the raw angle without wrapping.
   * 
   * @return angle in degrees
   */
  public double getAngle() {
    return angle;
  }

  /**
   * Gets the angle wrapped to [-180, 180] for display or comparison.
   * 
   * @return wrapped angle in degrees
   */
  public double getWrappedAngle() {
    return wrapTo180(angle);
  }

  // ===========================================================================================
  // Setters
  // ===========================================================================================

  /**
   * Sets the x component.
   * 
   * @param x new x component in meters
   */
  public void setX(double x) {
    this.x = x;
  }

  /**
   * Sets the y component.
   * 
   * @param y new y component in meters
   */
  public void setY(double y) {
    this.y = y;
  }

  /**
   * Sets the angle without wrapping.
   * 
   * @param angle new angle in degrees
   */
  public void setAngle(double angle) {
    this.angle = angle;
  }

  /**
   * Sets all pose components.
   * 
   * @param x new x component in meters
   * @param y new y component in meters
   * @param angle new angle in degrees
   */
  public void setPose(double x, double y, double angle) {
    this.x = x;
    this.y = y;
    this.angle = angle;
  }

  /**
   * Sets this pose to match another pose.
   * 
   * @param other the pose to copy from
   */
  public void setPose(Pose other) {
    this.x = other.getX();
    this.y = other.getY();
    this.angle = other.getAngle();
  }

  // ===========================================================================================
  // Utility Methods
  // ===========================================================================================

  /**
   * Wraps angle to [-180, 180] range.
   * 
   * @param angle angle to wrap
   * @return wrapped angle in degrees
   */
  private double wrapTo180(double angle) {
    while (angle <= -180) {
      angle += 360;
    }
    while (angle > 180) {
      angle -= 360;
    }
    return angle;
  }

  /**
   * Prints the pose with a prefix string.
   * 
   * @param prefix prefix string to display before pose values
   */
  public void print(String prefix) {
    System.out.printf("%s: x:%.2f y:%.2f a:%.2f\n", prefix, x, y, angle);
  }

  /**
   * Prints the pose with a prefix string and number.
   * 
   * @param prefix prefix string to display before pose values
   * @param index number to display with prefix
   */
  public void print(String prefix, int index) {
    System.out.printf("%s %d: x:%.2f y:%.2f a:%.2f\n", prefix, index, x, y, angle);
  }

  /**
   * Prints the pose with only a number prefix.
   * 
   * @param index number to display before pose values
   */
  public void print(int index) {
    System.out.printf("%d: x:%.2f y:%.2f a:%.2f\n", index, x, y, angle);
  }

  /**
   * Prints the pose without any prefix.
   */
  public void print() {
    System.out.printf("x:%.2f y:%.2f a:%.2f\n", x, y, angle);
  }
}