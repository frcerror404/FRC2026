package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class FaceHubCommand extends Command {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final ProfiledPIDController angleController;

  public FaceHubCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    Rotation2d desiredHeading = getDesiredHeading();
    double omega =
        angleController.calculate(drive.getRotation().getRadians(), desiredHeading.getRadians());

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  private Rotation2d getDesiredHeading() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Translation2d hubPosition = Constants.FieldConstants.getHubPosition(alliance);
    if (hubPosition == null) {
      return drive.getRotation();
    }
    Translation2d toHub = hubPosition.minus(drive.getPose().getTranslation());
    if (toHub.getNorm() < 1e-6) {
      return drive.getRotation();
    }
    return toHub.getAngle();
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
