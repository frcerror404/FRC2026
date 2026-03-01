package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;

public class FeedFuel extends SequentialCommandGroup {

  public FeedFuel(Feeder feeder, Hopper hopper) {
    super(feeder.getNewSetVoltsCommand(8.0), hopper.getNewSetVoltsCommand(3.0));
    addRequirements(feeder, hopper);
  }
}
