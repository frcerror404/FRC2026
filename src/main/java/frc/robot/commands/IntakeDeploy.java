package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeDeploy extends Command {

  private final Intake intake;

  public IntakeDeploy(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.pivotToIntake();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
