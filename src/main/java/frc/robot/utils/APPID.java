// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Custom PID Controller implementation for closed-loop control.
 * 
 * PID Control: Calculates output = P + I - D
 * - P (Proportional): Responds to current error
 * - I (Integral): Accumulates past errors to eliminate steady-state error
 * - D (Derivative): Predicts future error based on rate of change
 */
public class APPID {

  // ===========================================================================================
  // PID Coefficients
  // ===========================================================================================

  private double kP; // Proportional gain
  private double kI; // Integral gain
  private double kD; // Derivative gain

  // ===========================================================================================
  // Integral Control
  // ===========================================================================================

  private double iZone;          // Integral zone - prevents windup when far from target
  private double errorSum;       // Accumulated error for integral term
  private double errorIncrement; // Maximum error added per cycle (prevents integral windup)

  // ===========================================================================================
  // Setpoint and Error Tracking
  // ===========================================================================================

  private double desiredValue;    // Target value (setpoint)
  private double oldDesiredValue; // Previous setpoint (detects changes)
  private double previousValue;   // Last measured value (for derivative calculation)
  private double errorEpsilon;    // Acceptable error range for "done" check

  // ===========================================================================================
  // Output Limiting
  // ===========================================================================================

  private double maxOutput; // Maximum output value (default 1.0)

  // ===========================================================================================
  // State Tracking
  // ===========================================================================================

  private boolean firstCycle;   // True on first calculation (skips derivative)
  private int cycleCount;       // Consecutive cycles within epsilon range
  private int minCycleCount;    // Required cycles in range before "done"
  private final Timer pidTimer; // Tracks time between calculations

  // ===========================================================================================
  // Constructor
  // ===========================================================================================

  /**
   * Creates a new PID controller with specified gains and error tolerance.
   * 
   * @param p proportional gain
   * @param i integral gain
   * @param d derivative gain
   * @param epsilon acceptable error tolerance
   */
  public APPID(double p, double i, double d, double epsilon) {
    // PID gains
    this.kP = p;
    this.kI = i;
    this.kD = d;
    this.errorEpsilon = epsilon;

    // Initialize default values
    this.desiredValue = 0;
    this.oldDesiredValue = 0;
    this.previousValue = 0;
    this.firstCycle = true;
    this.maxOutput = 1.0;
    this.errorSum = 0;
    this.errorIncrement = 1;
    this.iZone = 0;

    // Completion tracking
    this.cycleCount = 0;
    this.minCycleCount = 10;

    // Timer for derivative calculation
    this.pidTimer = new Timer();
    this.pidTimer.start();
    this.pidTimer.reset();
  }

  // ===========================================================================================
  // Configuration Methods
  // ===========================================================================================

  /**
   * Updates the PID gains.
   * 
   * @param p proportional gain
   * @param i integral gain
   * @param d derivative gain
   */
  public void setConstants(double p, double i, double d) {
    this.kP = p;
    this.kI = i;
    this.kD = d;
  }

  /**
   * Sets the integral zone range.
   * Only accumulates integral error when within this range of the target.
   * Set to 0 to disable (integral always accumulates).
   * 
   * @param iZone maximum error for integral accumulation
   */
  public void setIZone(double iZone) {
    this.iZone = iZone;
  }

  /**
   * Sets the error tolerance for "done" detection.
   * 
   * @param epsilon acceptable error range
   */
  public void setErrorEpsilon(double epsilon) {
    this.errorEpsilon = epsilon;
  }

  /**
   * Sets the maximum error increment per cycle.
   * Limits how much error can be added to integral sum each cycle (prevents windup).
   * 
   * @param increment maximum error increment
   */
  public void setErrorIncrement(double increment) {
    this.errorIncrement = increment;
  }

  /**
   * Sets the target value (setpoint).
   * 
   * @param target desired value
   */
  public void setDesiredValue(double target) {
    this.desiredValue = target;
  }

  /**
   * Sets the maximum output magnitude.
   * 
   * @param max maximum output (0.0 to 1.0)
   */
  public void setMaxOutput(double max) {
    if (max >= 0.0 && max <= 1.0) {
      this.maxOutput = max;
    }
  }

  /**
   * Sets the minimum number of consecutive cycles within epsilon before "done".
   * 
   * @param cycles minimum cycle count
   */
  public void setMinDoneCycles(int cycles) {
    this.minCycleCount = cycles;
  }

  // ===========================================================================================
  // PID Calculation
  // ===========================================================================================

  /**
   * Calculates the PID output based on current measured value.
   * 
   * Control Logic:
   * 1. P: Proportional to current error (error = target - current)
   * 2. I: Sum of all past errors (with limits to prevent windup)
   * 3. D: Rate of change of measurement (opposes velocity)
   * 
   * Output = P + I - D (D is subtracted to dampen oscillations)
   * 
   * @param currentValue current measured value
   * @return PID output in range [-maxOutput, maxOutput]
   */
  public double calculate(double currentValue) {
    double pVal = 0.0;
    double iVal = 0.0;
    double dVal = 0.0;

    // First cycle: Initialize previous value, skip derivative
    if (firstCycle) {
      previousValue = currentValue;
      firstCycle = false;
      pidTimer.reset();
    }

    // Setpoint changed: Reset integral and state
    if (oldDesiredValue != desiredValue) {
      firstCycle = true;
      errorSum = 0;
    }

    // Calculate current error
    double error = desiredValue - currentValue;

    // P Term: Proportional to error
    pVal = kP * error;

    // I Term: Accumulate error with anti-windup protection
    if (Math.abs(error) > errorEpsilon) {
      // Only accumulate when outside acceptable range
      if (iZone == 0 || Math.abs(error) <= iZone) {
        // Limit error increment to prevent windup
        if (Math.abs(error) <= errorIncrement) {
          errorSum += error;
        } else {
          errorSum += Math.signum(error) * errorIncrement;
        }
      }
    } else {
      // Within acceptable range: Reset integral
      errorSum = 0;
    }
    iVal = kI * errorSum;

    // D Term: Rate of change (velocity) of measurement
    // Note: We use rate of measurement, not rate of error, to avoid derivative kick
    double dt = pidTimer.get();
    if (dt > 0 && !firstCycle) {
      double velocity = (currentValue - previousValue) / dt;
      dVal = kD * velocity;
    }

    // Calculate total output: P + I - D
    // D is subtracted because it opposes velocity toward target
    double output = pVal + iVal - dVal;

    // Clamp output to maximum
    output = Calculations.limitOutput(output, maxOutput);

    // Update state for next cycle
    previousValue = currentValue;
    oldDesiredValue = desiredValue;
    pidTimer.reset();

    return output;
  }

  /**
   * Legacy method name for backwards compatibility.
   * Calls calculate().
   * 
   * @param currentValue current measured value
   * @return PID output
   */
  public double calcPID(double currentValue) {
    return calculate(currentValue);
  }

  // ===========================================================================================
  // State Management
  // ===========================================================================================

  /**
   * Resets the accumulated integral error to zero.
   */
  public void resetErrorSum() {
    errorSum = 0;
  }

  /**
   * Checks if the controller has reached the target.
   * Returns true after staying within epsilon range for minimum cycle count.
   * 
   * @return true if target is reached and stable
   */
  public boolean isDone() {
    double currentError = Math.abs(desiredValue - previousValue);

    if (currentError <= errorEpsilon && !firstCycle) {
      cycleCount++;
      if (cycleCount >= minCycleCount) {
        cycleCount = 0;
        return true;
      }
    } else {
      cycleCount = 0; // Reset if we leave epsilon range
    }
    return false;
  }

  /**
   * Resets the PID controller to initial state.
   * Clears integral accumulation, cycle count, and state flags.
   */
  public void reset() {
    errorSum = 0;
    previousValue = 0;
    firstCycle = true;
    cycleCount = 0;
    pidTimer.reset();
  }

  // ===========================================================================================
  // Getters (for debugging/tuning)
  // ===========================================================================================

  /**
   * Gets the current error sum (integral term accumulator).
   * 
   * @return current integral error sum
   */
  public double getErrorSum() {
    return errorSum;
  }

  /**
   * Gets the current setpoint.
   * 
   * @return desired value
   */
  public double getSetpoint() {
    return desiredValue;
  }

  /**
   * Gets the last calculated error.
   * 
   * @return error = setpoint - previousValue
   */
  public double getError() {
    return desiredValue - previousValue;
  }
}