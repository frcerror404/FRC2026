// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FaceHubCommand;
import frc.robot.commands.FeedFuel;
import frc.robot.commands.FeedFuelReverse;
import frc.robot.commands.HopperToFeeder;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.IntakeFuelReverse;
import frc.robot.commands.IntakeStow;
import frc.robot.commands.Shoot;
import frc.robot.commands.StopFeederHopper;
import frc.robot.commands.StopHopper;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooterReverse.ShooterReverse;
import frc.robot.subsystems.shooterReverse.ShooterReverseIOTalonFX;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final ShooterReverse shooter1;
  private final Shooter shooter2;
  private final Shooter shooter3;
  private final Feeder feeder;
  private final Hopper hopper;
  private final Intake intake;
  // private final Limelight shooterLimelight;
  private final IntakePivot intakePivot = new IntakePivot(16);
  // private final Climber climber = new Climber(1);

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // private boolean m_TeleopInitialized = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //  CanDef.Builder canivoreCanBuilder = CanDef.builder().bus(CanBus.CANivore);
    CanDef.Builder rioCanBuilder = CanDef.builder().bus(CanBus.Rio);
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        shooter1 = new ShooterReverse(new ShooterReverseIOTalonFX(rioCanBuilder.id(4).build()));
        shooter2 = new Shooter(new ShooterIOTalonFX(rioCanBuilder.id(3).build()));
        shooter3 = new Shooter(new ShooterIOTalonFX(rioCanBuilder.id(7).build()));
        feeder = new Feeder(new FeederIOTalonFX(rioCanBuilder.id(6).build()));
        intake = new Intake(new IntakeIOTalonFX(rioCanBuilder.id(14).build()));
        hopper = new Hopper(new HopperIOTalonFX(rioCanBuilder.id(12).build()));
        // shooterLimelight = new Limelight("limelight-shooter");

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        shooter1 = null;
        shooter2 = null;
        shooter3 = null;
        feeder = null;
        intake = null;
        hopper = null;
        // shooterLimelight = null;

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        shooter1 = null;
        shooter2 = null;
        shooter3 = null;

        feeder = null;
        intake = null;
        hopper = null;
        // shooterLimelight = null;

        break;
    }

    registerNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Driver Controls
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driver.rightTrigger().whileTrue(new IntakeFuel(intake)).whileFalse(new StopIntake(intake));
    driver
        .leftTrigger()
        .whileTrue(new IntakeFuelReverse(intake))
        .whileFalse(new StopIntake(intake));

    // Operator Controls
    operator
        .start()
        .toggleOnTrue(
            new FaceHubCommand(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX()));
    operator.leftTrigger().whileTrue(new HopperToFeeder(hopper)).whileFalse(new StopHopper(hopper));
    operator.a().onTrue(new IntakeDeploy(intakePivot));
    operator.b().onTrue(new IntakeStow(intakePivot));
    operator
        .rightTrigger()
        .onTrue(new Shoot(shooter1, shooter2, shooter3))
        .onFalse(new StopShooter(shooter1, shooter2, shooter3));
    operator
        .rightBumper()
        .whileTrue(new FeedFuel(feeder, hopper))
        .whileFalse(new StopFeederHopper(feeder, hopper));
    operator
        .leftBumper()
        .whileTrue(new FeedFuelReverse(feeder, hopper))
        .whileFalse(new StopFeederHopper(feeder, hopper));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // Register commands for auto builder
  private void registerNamedCommands() {
    NamedCommands.registerCommand("IntakeFuel", new IntakeFuel(intake));
    NamedCommands.registerCommand("StopIntake", new StopIntake(intake));
    NamedCommands.registerCommand("DropIntake", new IntakeDeploy(intakePivot));
    NamedCommands.registerCommand("FeedFuel", new FeedFuel(feeder, hopper));
    NamedCommands.registerCommand("StopFeederHopper", new StopFeederHopper(feeder, hopper));
    NamedCommands.registerCommand("ShootFuel", new Shoot(shooter1, shooter2, shooter3));
    NamedCommands.registerCommand("StopShooter", new StopShooter(shooter1, shooter2, shooter3));
  }

  // public void teleopInit() {
  //   if (!this.m_TeleopInitialized) {
  //     // Only want to initialize starting position once (if teleop multiple times dont reset pose
  //     // again)
  //     //   vision.updateStartingPosition();
  //     // Turn on updating odometry based on Apriltags
  //     //   vision.enableUpdateOdometryBasedOnApriltags();
  //     m_TeleopInitialized = true;
  //     SignalLogger.setPath("/media/sda1/");
  //     SignalLogger.start();
  //   }
  // }
}
