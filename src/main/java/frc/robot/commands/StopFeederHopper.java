package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;

public class StopFeederHopper extends SequentialCommandGroup {

  public StopFeederHopper(Feeder feeder1, Feeder feeder2, Hopper hopper) {
    super(feeder1.getStopCommand(), feeder2.getStopCommand(), hopper.getStopCommand());
    addRequirements(feeder1, feeder2, hopper);
  }
}
