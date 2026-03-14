package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;

public class FeedFuelReverse extends SequentialCommandGroup {

  public FeedFuelReverse(Feeder feeder1, Feeder feeder2, Hopper hopper) {
    super(feeder1.runFeeder(-6.0), feeder2.runFeeder(-6.0), hopper.runHopper(-6.0));
    addRequirements(feeder1, feeder2, hopper);
  }
}
