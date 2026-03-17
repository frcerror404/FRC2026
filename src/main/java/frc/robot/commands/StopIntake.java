package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class StopIntake extends SequentialCommandGroup {

  public StopIntake(Intake intake1, Intake intake2) {
    super(intake1.getStopCommand(), intake2.getStopCommand());
    addRequirements(intake1, intake2);
  }
}
