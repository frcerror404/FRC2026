package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class IntakeFuelReverse extends SequentialCommandGroup {

  public IntakeFuelReverse(Intake intake1, Intake intake2) {
    super(intake1.runIntake(5), intake2.runIntake(-5));
    addRequirements(intake1, intake2);
  }
}
