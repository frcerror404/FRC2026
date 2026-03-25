package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAndFeed extends SequentialCommandGroup {

  public ShootAndFeed(Hopper hopper, Feeder feeder, Shooter shooter) {
    super(hopper.runHopper(-6), feeder.runFeeder(-6), shooter.shootFuel(-6));
    addRequirements(hopper, feeder, shooter);
  }
}
