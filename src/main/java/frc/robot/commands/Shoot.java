package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterReverse.ShooterReverse;

public class Shoot extends SequentialCommandGroup {

  public Shoot(ShooterReverse shooter1, Shooter shooter2, Shooter shooter3) {
    super(
        shooter1.getNewSetVoltsCommand(2),
        shooter2.getNewSetVoltsCommand(2),
        shooter3.getNewSetVoltsCommand(2));
    addRequirements(shooter1, shooter2, shooter3);
  }
}
