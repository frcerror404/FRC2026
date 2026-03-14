package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;

public class FeedFuel extends SequentialCommandGroup {

  public FeedFuel(Feeder feeder1, Feeder feeder2, Hopper hopper) {
    super(feeder1.runFeeder(9), feeder2.runFeeder(9), hopper.runHopper(9));
    addRequirements(feeder1, feeder2, hopper);
  }
}
