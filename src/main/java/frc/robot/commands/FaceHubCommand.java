// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.DoubleSupplier;
import limelight.Limelight;
import limelight.networktables.LimelightTargetData;

public class FaceHubCommand extends Command {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final int[] RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
  private static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final LimelightTargetData targetData;
  private final Set<Subsystem> requirements;
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          ANGLE_KP,
          0.0,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
  private Rotation2d lastTargetRotation = Rotation2d.kZero;

  public FaceHubCommand(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Limelight limelight) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetData = limelight == null ? null : limelight.getData().targetData;
    this.requirements = Set.of(drive);

    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    lastTargetRotation = drive.getRotation();
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    Rotation2d targetRotation = getTargetRotation();
    double omega =
        angleController.calculate(drive.getRotation().getRadians(), targetRotation.getRadians());

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Rotation2d getTargetRotation() {
    Rotation2d fallback = lastTargetRotation != null ? lastTargetRotation : drive.getRotation();
    if (targetData == null || !targetData.getTargetStatus()) {
      lastTargetRotation = fallback;
      return fallback;
    }

    int tagId = (int) targetData.getAprilTagID();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (!isHubTagId(tagId, alliance)) {
      lastTargetRotation = fallback;
      return fallback;
    }

    double txDegrees = targetData.getHorizontalOffsetFromPrincipal();
    lastTargetRotation = drive.getRotation().minus(Rotation2d.fromDegrees(txDegrees));
    return lastTargetRotation;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  private static boolean isHubTagId(int tagId, Alliance alliance) {
    int[] tagIds = alliance == Alliance.Red ? RED_HUB_TAG_IDS : BLUE_HUB_TAG_IDS;
    for (int id : tagIds) {
      if (id == tagId) {
        return true;
      }
    }
    return false;
  }
}
