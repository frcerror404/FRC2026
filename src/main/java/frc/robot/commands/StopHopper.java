package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hopper.Hopper;

public class StopHopper extends SequentialCommandGroup {

  public StopHopper(Hopper hopper) {
    super(hopper.getStopCommand());
    addRequirements(hopper);
  }
}
