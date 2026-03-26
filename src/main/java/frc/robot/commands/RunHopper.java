package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hopper.Hopper;

public class RunHopper extends SequentialCommandGroup {

  public RunHopper(Hopper hopper) {
    super(hopper.runHopper(10));
    addRequirements(hopper);
  }
}
