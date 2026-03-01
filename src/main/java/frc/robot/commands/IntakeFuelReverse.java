package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class IntakeFuelReverse extends SequentialCommandGroup {

  public IntakeFuelReverse(Intake intake) {
    super(intake.getNewSetVoltsCommand(-2.0));
    addRequirements(intake);
  }
}
