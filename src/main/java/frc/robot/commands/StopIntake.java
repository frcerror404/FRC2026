package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class StopIntake extends SequentialCommandGroup {

  public StopIntake(Intake intake) {
    super(intake.getStopCommand());
    addRequirements(intake);
  }
}
