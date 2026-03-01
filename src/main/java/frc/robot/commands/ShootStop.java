package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class ShootStop extends SequentialCommandGroup {

  public ShootStop(Shooter shooter) {
    super(shooter.getNewSetVoltsCommand(0));
  }
}
