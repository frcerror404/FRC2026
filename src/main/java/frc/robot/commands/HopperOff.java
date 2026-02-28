package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;

public class HopperOff extends Command {

  private final Hopper hopper;

  public HopperOff(Hopper hopper) {
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.stopHopper();
  }
}
