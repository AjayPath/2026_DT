// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final TalonFX m_drivingKraken;
  private final SparkMax m_turningSpark;

  private final AbsoluteEncoder m_turningEncoder;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private final VelocityVoltage m_drivingVelocityRequest;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingKraken = new TalonFX(drivingCANId);
    m_drivingKraken.getConfigurator().apply(Configs.MAXSwerveModule.drivingConfig);
    m_drivingVelocityRequest = new VelocityVoltage(0).withSlot(0);  
    
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
    
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingKraken.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.

    double velocityWheelRPS = m_drivingKraken.getVelocity().getValueAsDouble();

    double velocityMetersPerSec = velocityWheelRPS * ModuleConstants.kWheelCircumferenceMeters;

    return new SwerveModuleState(
      velocityMetersPerSec,
      new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    double positionWheelRotations = m_drivingKraken.getPosition().getValueAsDouble();

    double positionMeters = positionWheelRotations * ModuleConstants.kWheelCircumferenceMeters;

    return new SwerveModulePosition(
        positionMeters,
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    double targetVelocityWheelRPS = correctedDesiredState.speedMetersPerSecond / ModuleConstants.kWheelCircumferenceMeters;
    m_drivingKraken.setControl(m_drivingVelocityRequest.withVelocity(targetVelocityWheelRPS));
    //m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingKraken.setPosition(0);
  }

  public void stop() {
    m_drivingKraken.stopMotor();
    m_turningSpark.stopMotor();
  }
}