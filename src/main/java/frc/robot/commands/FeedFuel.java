package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;

public class FeedFuel extends SequentialCommandGroup {

  public FeedFuel(Feeder feeder, Hopper hopper) {
    super(feeder.runFeeder(10), hopper.runHopper(10));
    addRequirements(feeder, hopper);
  }
}
