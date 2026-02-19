package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

public class TurnToAngle extends Command {
  private final DriveSubsystem driveSubsystem;
  private final APPID turnPID;

  private final double targetAngleDeg;     // desired heading, degrees
  private final double angleToleranceDeg;  // finish tolerance, degrees

  // Tune these like you did in DriveToPoint (start with just P)
  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.0;
  private static final double kMaxRot = 0.25; // normalized [-1..1]

  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngleDeg, double angleToleranceDeg) {
    this.driveSubsystem = driveSubsystem;
    this.targetAngleDeg = targetAngleDeg;
    this.angleToleranceDeg = angleToleranceDeg;

    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleToleranceDeg);
    this.turnPID.setMaxOutput(kMaxRot);

    addRequirements(driveSubsystem);
  }

  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngleDeg) {
    this(driveSubsystem, targetAngleDeg, 2.0);
  }

  @Override
  public void initialize() {
    turnPID.reset();
  }

  @Override
  public void execute() {
    // Your odometry pose returns angle normalized to [0, 360) :contentReference[oaicite:1]{index=1}
    Pose currentPose = driveSubsystem.getPose();

    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle  = Calculations.normalizeAngle360(targetAngleDeg);

    // Signed shortest path error in [-180, 180] :contentReference[oaicite:2]{index=2}
    double angleError = Calculations.shortestAngularDistance(targetAngle, currentAngle);

    // Same sign convention you used in DriveToPoint :contentReference[oaicite:3]{index=3}
    turnPID.setDesiredValue(0);
    double rotCmd = turnPID.calculate(-angleError);

    // Turn in place: no translation
    driveSubsystem.drive(0.0, 0.0, rotCmd, true);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    Pose currentPose = driveSubsystem.getPose();

    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle  = Calculations.normalizeAngle360(targetAngleDeg);

    double angleError = Math.abs(Calculations.shortestAngularDistance(targetAngle, currentAngle));
    return angleError <= angleToleranceDeg;
  }
}