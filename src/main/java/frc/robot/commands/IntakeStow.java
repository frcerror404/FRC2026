package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class IntakeStow extends Command {

  private final IntakePivot intakeStow;

  public IntakeStow(IntakePivot intakePivot) {
    this.intakeStow = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    intakeStow.pivotToStow();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
