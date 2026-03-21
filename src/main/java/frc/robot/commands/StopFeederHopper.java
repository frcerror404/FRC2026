package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;

public class StopFeederHopper extends SequentialCommandGroup {

  public StopFeederHopper(Feeder feeder, Hopper hopper) {
    super(feeder.getStopCommand(), hopper.getStopCommand());
    addRequirements(feeder, hopper);
  }
}
