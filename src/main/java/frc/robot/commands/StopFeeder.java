package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;

public class StopFeeder extends SequentialCommandGroup {

  public StopFeeder(Feeder feeder) {
    super(feeder.getStopCommand());
    addRequirements(feeder);
  }
}
