package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAndFeed extends SequentialCommandGroup {

  public ShootAndFeed(
      Hopper hopper,
      Feeder feeder1,
      Feeder feeder2,
      Shooter shooter1,
      Shooter shooter2,
      Shooter shooter3,
      Shooter shooter4) {
    super(
        hopper.runHopper(6),
        feeder1.runFeeder(6),
        feeder2.runFeeder(6),
        shooter1.shootFuel(8, true),
        shooter2.shootFuel(8, true),
        shooter3.shootFuel(8, false),
        shooter4.shootFuel(8, false));
    addRequirements(hopper, feeder1, feeder2, shooter1, shooter2, shooter3, shooter4);
  }
}
