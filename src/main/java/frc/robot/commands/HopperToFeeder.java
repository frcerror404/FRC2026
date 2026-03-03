package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hopper.Hopper;

public class HopperToFeeder extends SequentialCommandGroup {

  public HopperToFeeder(Hopper hopper) {
    super(hopper.runHopper(3.0));
    addRequirements(hopper);
  }
}
