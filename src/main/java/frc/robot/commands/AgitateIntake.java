package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class AgitateIntake extends Command {

  private final IntakePivot intake;

  public AgitateIntake(IntakePivot intakePivot) {
    this.intake = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    intake.agitateIntake();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
