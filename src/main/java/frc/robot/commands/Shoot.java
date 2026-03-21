package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends SequentialCommandGroup {

  public Shoot(Shooter shooter) {
    super(
        shooter.shootFuel(7)
    );
    addRequirements(shooter);
  }
}
