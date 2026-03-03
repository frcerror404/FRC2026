package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterReverse.ShooterReverse;

public class Shoot extends SequentialCommandGroup {

  public Shoot(ShooterReverse shooter1, Shooter shooter2, Shooter shooter3) {
    super(shooter1.shootFuel(5.0), shooter2.shootFuel(5.0), shooter2.shootFuel(5.0));
    addRequirements(shooter1, shooter2, shooter3);
  }
}
