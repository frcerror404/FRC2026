package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterOut extends Command {

  private final Shooter shooter;

  public ShooterOut(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.runShooter();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
