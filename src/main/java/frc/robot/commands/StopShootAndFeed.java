package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterReverse.ShooterReverse;

public class StopShootAndFeed extends SequentialCommandGroup {

  public StopShootAndFeed(
      Hopper hopper, Feeder feeder, ShooterReverse shooter1, Shooter shooter2, Shooter shooter3) {
    super(
        hopper.getNewSetVoltsCommand(0),
        feeder.getNewSetVoltsCommand(0),
        shooter1.shootFuel(0),
        shooter2.shootFuel(0),
        shooter3.shootFuel(0));
    addRequirements(shooter1, shooter2, shooter3);
  }
}
