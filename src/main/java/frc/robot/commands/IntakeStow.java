package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class IntakeStow extends Command {

  private final IntakePivot intakePivot;

  public IntakeStow(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    intakePivot.pivotToStow();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
