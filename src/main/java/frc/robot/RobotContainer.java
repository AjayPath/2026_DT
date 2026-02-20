// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.Pose;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

        // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.a().whileTrue(aimToAllowedTagOnce());

    m_driverController.x().whileTrue(new DriveToPoint(m_robotDrive, 0, 0, 0));

    m_driverController.leftBumper().onTrue(
        new InstantCommand(() -> {

            Pose visionPose = m_limelight.getPoseFromTag(
                m_robotDrive.getPoseContinuous().getAngle()
            );

            if (visionPose != null) {
                m_robotDrive.resetOdometry(visionPose);
            }

        }, m_robotDrive)
    );

  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  /////////////////////////////////////////////////////////////////////////
  
  /** Snapshots tx once and turns until centered (tx -> 0), only for tags 2/5/10. */
  private Command aimToAllowedTagOnce() {
    return Commands.defer(
        () -> {
          // Gate: must see a target and it must be an allowed tag
          if (!m_limelight.hasValidTarget() || !m_limelight.isTagAllowedForRotation()) {
            return Commands.none();
          }

          double txDeg = m_limelight.getTX();

          // Use continuous heading for control math (not the wrapped [0..360) one)
          double currentYawDeg = m_robotDrive.getPoseContinuous().getAngle();

          // Typical mapping: targetYaw = currentYaw - tx
          // If it turns the wrong way on your robot, flip the sign to +tx.
          double targetYawDeg = currentYawDeg - txDeg;

          return new TurnToAngle(m_robotDrive, targetYawDeg);
        },
        Set.of(m_robotDrive) // requirements for the *returned* command
    );
  }

  /////////////////////////////////////////////////////////////////////////

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
