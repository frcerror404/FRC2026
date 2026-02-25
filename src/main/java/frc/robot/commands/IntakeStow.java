package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeStow extends Command {

  private final Intake intake;

  public IntakeStow(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.pivotToStow();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
