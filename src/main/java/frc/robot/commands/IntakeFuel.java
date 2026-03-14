package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class IntakeFuel extends SequentialCommandGroup {

  public IntakeFuel(Intake intake1, Intake intake2) {
    super(intake1.runIntake(10), intake2.runIntake(-10));
    addRequirements(intake1, intake2);
  }
}
