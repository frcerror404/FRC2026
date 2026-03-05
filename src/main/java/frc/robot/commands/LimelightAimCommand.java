package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * Rotates the robot to center the Limelight on the hub AprilTags (IDs 25/26 on blue, 9/10 on red)
 * using a simple PID on tx. Translation is still driver-controlled.
 */
public class LimelightAimCommand extends Command {
  private static final double DEADBAND = 0.1;

  // Tune these gains on the robot
  private static final double AIM_KP = 0.1;
  private static final double AIM_KI = 0.0;
  private static final double AIM_KD = 0.005;

  // tx threshold to consider "on target" (degrees)
  private static final double ON_TARGET_THRESHOLD_DEG = 1.0;

  private static final Set<Integer> BLUE_TAG_IDS = Set.of(25, 26);
  private static final Set<Integer> RED_TAG_IDS = Set.of(9, 10);

  private final Drive drive;
  private final Vision vision;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final PIDController aimController;

  public LimelightAimCommand(
      Drive drive, Vision vision, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.vision = vision;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    aimController = new PIDController(AIM_KP, AIM_KI, AIM_KD);
    aimController.setTolerance(ON_TARGET_THRESHOLD_DEG);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    aimController.reset();
    aimController.setSetpoint(0.0); // Target tx = 0 (centered)
  }

  @Override
  public void execute() {
    // Only aim if we see a valid hub tag for our alliance
    boolean shouldAim = isValidHubTarget();

    double omega = 0.0;
    if (shouldAim) {
      double tx = vision.getTargetX(0).getDegrees();
      omega = -aimController.calculate(tx);
    }

    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                linearVelocity.getX() * Drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * Drive.getMaxLinearSpeedMetersPerSec(),
                omega),
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  private boolean isValidHubTarget() {
    if (!vision.hasTarget(0)) return false;
    int tagId = vision.getPrimaryTagId(0);
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Blue ? BLUE_TAG_IDS.contains(tagId) : RED_TAG_IDS.contains(tagId);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }
}
