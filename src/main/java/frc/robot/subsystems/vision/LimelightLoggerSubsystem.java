package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightLogger;
import limelight.Limelight;

public class LimelightLoggerSubsystem extends SubsystemBase {
  private final Limelight limelight;

  public LimelightLoggerSubsystem(Limelight limelight) {
    this.limelight = limelight;
  }

  @Override
  public void periodic() {
    LimelightLogger.log(limelight);
  }
}
