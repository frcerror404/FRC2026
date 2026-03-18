package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends SequentialCommandGroup {

  public Shoot(Shooter shooter1, Shooter shooter2, Shooter shooter3, Shooter shooter4) {
    super(
        shooter1.shootFuel(-7, true),
        shooter2.shootFuel(-7, true),
        shooter3.shootFuel(7, true),
        shooter4.shootFuel(7, true));
    addRequirements(shooter1, shooter2, shooter3, shooter4);
  }
}
