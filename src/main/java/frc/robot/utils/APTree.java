// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

/**
 * Wrapper class for WPILib's InterpolatingTreeMap with simplified interface.
 * 
 * Purpose:
 * - Stores key-value pairs with automatic linear interpolation
 * - Returns interpolated values for keys between stored data points
 * - Useful for lookup tables (distance-to-angle, distance-to-speed, etc.)
 * 
 * How Interpolation Works:
 * - If key exists exactly: Returns the stored value
 * - If key is between two points: Linearly interpolates between them
 * - If key is outside range: Returns nearest boundary value
 * 
 * Example Use Case:
 * - Store shooter angles for different distances: {(1.0m, 30°), (2.0m, 45°), (3.0m, 55°)}
 * - Request angle at 1.5m: Automatically returns 37.5° (interpolated)
 * - Request angle at 2.7m: Returns 52° (interpolated between 45° and 55°)
 */
public class APTree {

  // ===========================================================================================
  // Interpolating Map
  // ===========================================================================================

  private final InterpolatingTreeMap<Double, Double> treeMap;

  // ===========================================================================================
  // Constructor
  // ===========================================================================================

  /**
   * Creates a new interpolating lookup table.
   */
  public APTree() {
    treeMap = new InterpolatingDoubleTreeMap();
  }

  // ===========================================================================================
  // Data Management
  // ===========================================================================================

  /**
   * Inserts multiple key-value pairs into the lookup table.
   * 
   * Array Format: {{key1, value1}, {key2, value2}, ...}
   * 
   * Example:
   * <pre>
   * double[][] shooterData = {
   *     {1.0, 30.0},  // At 1.0m, angle is 30°
   *     {2.0, 45.0},  // At 2.0m, angle is 45°
   *     {3.0, 55.0}   // At 3.0m, angle is 55°
   * };
   * tree.InsertValues(shooterData);
   * </pre>
   * 
   * @param values 2D array where each row is [key, value]
   */
  public void InsertValues(double[][] values) {
    for (double[] pair : values) {
      treeMap.put(pair[0], pair[1]);
    }
  }

  // ===========================================================================================
  // Value Lookup
  // ===========================================================================================

  /**
   * Gets the value for a given key with automatic interpolation.
   * 
   * Behavior:
   * - Exact match: Returns stored value
   * - Between points: Returns linearly interpolated value
   * - Outside range: Returns nearest boundary value
   * 
   * @param key the lookup key
   * @return interpolated value at the key
   */
  public double GetValue(double key) {
    return treeMap.get(key);
  }
}