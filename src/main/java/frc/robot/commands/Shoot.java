package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends SequentialCommandGroup {

  public Shoot(Shooter shooter, Shooter shooter2, Shooter shooter3) {
    super(
        shooter.getNewSetVoltsCommand(-5.5),
        shooter2.getNewSetVoltsCommand(5.5),
        shooter3.getNewSetVoltsCommand(5.5));
    addRequirements(shooter, shooter2, shooter3);
  }
}
