package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class AgitateIntake extends Command {

  private final IntakePivot intakePivot;

  public AgitateIntake(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    intakePivot.agitateIntake();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
