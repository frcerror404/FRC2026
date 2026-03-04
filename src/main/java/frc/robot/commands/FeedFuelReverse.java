package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;

public class FeedFuelReverse extends SequentialCommandGroup {

  public FeedFuelReverse(Feeder feeder, Hopper hopper) {
    super(feeder.runFeeder(-3.0), hopper.runHopper(-3.0));
    addRequirements(feeder, hopper);
  }
}
