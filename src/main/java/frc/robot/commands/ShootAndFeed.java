package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterReverse.ShooterReverse;

public class ShootAndFeed extends SequentialCommandGroup {

  public ShootAndFeed(
      Hopper hopper, Feeder feeder, ShooterReverse shooter1, Shooter shooter2, Shooter shooter3) {
    super(
        hopper.getNewSetVoltsCommand(4.0),
        feeder.getNewSetVoltsCommand(4.0),
        shooter1.shootFuel(8),
        shooter2.shootFuel(8),
        shooter3.shootFuel(8));
    addRequirements(shooter1, shooter2, shooter3);
  }
}
