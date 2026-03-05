package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlignToHub extends Command {

  private final AprilTagFieldLayout layout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
  private final Drive drive;
  private final Vision vision;

  public AlignToHub(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;

    addRequirements(drive);
  }

  @Override
  public void execute() {

    Pose2d hubPose;

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {

      hubPose = layout.getTagPose(10).get().toPose2d();

    } else {

      hubPose = layout.getTagPose(26).get().toPose2d();
    }

    Translation2d hubTranslation = hubPose.getTranslation();
    Pose2d robotPose = drive.getPose();

    // Vector from robot to hub
    Translation2d robotToHub = hubTranslation.minus(robotPose.getTranslation());

    // Desired heading
    Rotation2d targetHeading = robotToHub.getAngle();

    // Heading error
    double headingError = targetHeading.minus(robotPose.getRotation()).getRadians();

    // Proportional control
    double turnSpeed = MathUtil.clamp(headingError * 3.0, -2.0, 2.0);

    // Run drivetrain
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, turnSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
