package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class IntakeDeploy extends Command {

  private final IntakePivot intakePivot;

  public IntakeDeploy(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    intakePivot.pivotToIntake();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
