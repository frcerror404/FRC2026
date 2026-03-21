package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class StopShooter extends SequentialCommandGroup {

  public StopShooter(Shooter shooter) {
    super(
        shooter.getStopCommand());
    addRequirements(shooter);
  }
}
