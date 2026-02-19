// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a 2D vector with x, y components and polar coordinates (magnitude and angle).
 * Supports vector operations including addition, subtraction, and coordinate transformations.
 * All distance measurements are in meters.
 */
public class Vector {
  
  private double x = 0.0;
  private double y = 0.0;
  private double mag = 0.0;
  private Rotation2d angle = Rotation2d.fromDegrees(0);

  // ===========================================================================================
  // Constructors
  // ===========================================================================================

  /**
   * Creates a vector from Cartesian coordinates.
   * Calculates magnitude and angle from x and y components.
   * 
   * @param x the x component (meters)
   * @param y the y component (meters)
   */
  public Vector(double x, double y) {
    this.x = x;
    this.y = y;
    this.angle = new Rotation2d(Math.atan2(y, x));
    this.mag = Math.sqrt((Math.pow(x, 2)) + Math.pow(y, 2));
  }

  /**
   * Creates a vector from Cartesian coordinates with fallback angle.
   * Uses provided angle if both x and y are effectively zero.
   * 
   * @param x the x component (meters)
   * @param y the y component (meters)
   * @param angle fallback angle if x and y are both zero
   */
  public Vector(double x, double y, Rotation2d angle) {
    this.x = x;
    this.y = y;

    // Use provided angle if vector is at origin
    if (Math.abs(x) < 1e-9 && Math.abs(y) < 1e-9) {
      this.angle = angle;
    } else {
      this.angle = new Rotation2d(Math.atan2(y, x));
    }
    this.mag = Math.sqrt((Math.pow(x, 2)) + Math.pow(y, 2));
  }

  /**
   * Creates a vector from polar coordinates.
   * Calculates x and y components from magnitude and angle.
   * 
   * @param mag the magnitude (meters)
   * @param angle the angle
   */
  public Vector(double mag, Rotation2d angle) {
    this.mag = mag;
    this.angle = Calculations.normalizeAngle(angle);
    this.x = mag * Math.cos(angle.getRadians());
    this.y = mag * Math.sin(angle.getRadians());
  }

  /**
   * Default constructor for empty vector.
   */
  Vector() {}

  // ===========================================================================================
  // Vector Operations
  // ===========================================================================================

  /**
   * Subtracts a vector from this vector and returns the result.
   * 
   * @param v the vector to subtract
   * @return new vector representing the difference
   */
  public Vector subtractVector(Vector v) {
    x -= v.getX();
    y -= v.getY();
    angle = new Rotation2d(Math.atan2(y, x));
    mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    return new Vector(x, y);
  }

  /**
   * Adds a vector to this vector and returns the result.
   * 
   * @param v the vector to add
   * @return new vector representing the sum
   */
  public Vector addVector(Vector v) {
    x += v.getX();
    y += v.getY();
    angle = new Rotation2d(Math.atan2(y, x));
    mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    return new Vector(x, y);
  }

  /**
   * Transforms the vector from robot-relative to field-relative coordinates.
   * 
   * @param gyroAngle the robot's heading in degrees
   * @return transformed vector in field coordinates
   */
  public Vector transform(double gyroAngle) {
    angle = angle.plus(Rotation2d.fromDegrees(gyroAngle));
    x = mag * Math.cos(angle.getRadians());
    y = mag * Math.sin(angle.getRadians());
    return new Vector(x, y);
  }

  /**
   * Reflects the vector across the y-axis.
   */
  public void reflectY() {
    x = -x;
    angle = new Rotation2d(Math.atan2(Math.sin(angle.getRadians()), -Math.cos(angle.getRadians())));
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
   * Gets the magnitude.
   * 
   * @return magnitude in meters
   */
  public double getMagnitude() {
    return mag;
  }

  /**
   * Gets the angle.
   * 
   * @return angle as Rotation2d
   */
  public Rotation2d getAngle() {
    return angle;
  }

  // ===========================================================================================
  // Setters
  // ===========================================================================================

  /**
   * Sets the x component and recalculates angle and magnitude.
   * 
   * @param x new x component in meters
   */
  public void setX(double x) {
    this.x = x;
    this.angle = new Rotation2d(Math.atan2(y, x));
    this.mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  /**
   * Sets the y component and recalculates angle and magnitude.
   * 
   * @param y new y component in meters
   */
  public void setY(double y) {
    this.y = y;
    this.angle = new Rotation2d(Math.atan2(y, x));
    this.mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  /**
   * Sets the magnitude and recalculates x and y components.
   * 
   * @param mag new magnitude in meters
   */
  public void setMagnitude(double mag) {
    this.x = mag * Math.cos(angle.getRadians());
    this.y = mag * Math.sin(angle.getRadians());
    this.mag = mag;
  }

  /**
   * Sets the angle and recalculates x and y components.
   * 
   * @param angle new angle
   */
  public void setAngle(Rotation2d angle) {
    this.x = mag * Math.cos(angle.getRadians());
    this.y = mag * Math.sin(angle.getRadians());
    this.angle = angle;
  }

  // ===========================================================================================
  // Utility Methods
  // ===========================================================================================

  /**
   * Prints the vector with a prefix string.
   * 
   * @param prefix prefix string to display before vector values
   */
  public void print(String prefix) {
    System.out.printf("%s: x:%.3f y:%.3f a:%.3f\n", prefix, x, y, angle.getDegrees());
  }

  /**
   * Prints the vector without a prefix.
   */
  public void print() {
    System.out.printf("x:%.3f y:%.3f a:%.3f\n", x, y, angle.getDegrees());
  }
}