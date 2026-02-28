package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;

public class HopperOn extends Command {

  private final Hopper hopper;

  public HopperOn(Hopper hopper) {
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.runHopper();
  }
}
