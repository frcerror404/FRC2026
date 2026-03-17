// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
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
import frc.robot.commands.AgitateIntake;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedFuel;
import frc.robot.commands.FeedFuelReverse;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.IntakeFuelReverse;
import frc.robot.commands.IntakeStow;
import frc.robot.commands.LimelightAimCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAndFeed;
import frc.robot.commands.StopFeederHopper;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShootAndFeed;
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
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
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

  private final Shooter shooter1;
  private final Shooter shooter2;
  private final Shooter shooter3;
  private final Shooter shooter4;
  private final Feeder feeder1;
  private final Feeder feeder2;
  private final Hopper hopper;
  private final Intake intake1;
  private final Intake intake2;

  private final Vision vision;

  // Intake
  // (Roller ID, Pivot ID)
  private final IntakePivot intakePivot = new IntakePivot(16);
  //   private final Climber climber = new Climber(60);

  // CANdle
  private final CANdle candle = new CANdle(50, "2026 Swerve"); // climber

  // private final Climber climber = new Climber(1);

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  // private final Limelight limelight1 =
  //     Constants.currentMode == Constants.Mode.REAL ? new Limelight("limelight1") : null;
  // private final LimelightLoggerSubsystem limelightLoggerSubsystem;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private boolean slowMode = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // limelightLoggerSubsystem = new LimelightLoggerSubsystem(limelight1);

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

        shooter1 = new Shooter(new ShooterIOTalonFX(rioCanBuilder.id(4).build()));
        shooter2 = new Shooter(new ShooterIOTalonFX(rioCanBuilder.id(3).build()));
        shooter3 = new Shooter(new ShooterIOTalonFX(rioCanBuilder.id(7).build()));
        shooter4 = new Shooter(new ShooterIOTalonFX(rioCanBuilder.id(9).build()));

        feeder1 = new Feeder(new FeederIOTalonFX(rioCanBuilder.id(6).build()));
        feeder2 = new Feeder(new FeederIOTalonFX(rioCanBuilder.id(11).build()));
        intake1 = new Intake(new IntakeIOTalonFX(rioCanBuilder.id(14).build()));
        intake2 = new Intake(new IntakeIOTalonFX(rioCanBuilder.id(15).build()));
        hopper = new Hopper(new HopperIOTalonFX(rioCanBuilder.id(12).build()));

        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.shooterLimelightName, drive::getRotation));
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
        shooter4 = null;

        feeder1 = null;
        feeder2 = null;
        intake1 = null;
        intake2 = null;
        hopper = null;

        vision = null;

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
        shooter4 = null;

        feeder1 = null;
        feeder2 = null;
        intake1 = null;
        intake2 = null;
        hopper = null;

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    // Set up auto routines

    registerNamedCommands();
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

    candle.setControl(
        new SolidColor(0, 8) // LEDs 0-7 (the built-in ones)
            .withColor(new RGBWColor(128, 0, 128)));

    // Register Commands for Auton
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // D-pad left toggles slow mode (30% speed)
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY() * (slowMode ? 0.3 : 1.0),
            () -> -driver.getLeftX() * (slowMode ? 0.3 : 1.0),
            () -> -driver.getRightX() * (slowMode ? 0.3 : 1.0)));

    driver.povLeft().onTrue(Commands.runOnce(() -> slowMode = !slowMode));

    // Reset 0
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Driver - Run Intake
    driver
        .rightTrigger()
        .onTrue(new IntakeFuel(intake1, intake2))
        .onFalse(new StopIntake(intake1, intake2));

    // Driver - Reverse Intake
    driver
        .leftTrigger()
        .whileTrue(new IntakeFuelReverse(intake1, intake2))
        .whileFalse(new StopIntake(intake1, intake2));

    // Operator - Limelight Aim at Hub (tags 25/26 blue, 9/10 red)
    driver
        .a()
        .whileTrue(
            new LimelightAimCommand(
                drive, vision, () -> -driver.getLeftY(), () -> -driver.getLeftX()));

    // Operator - Deploy Intake
    operator.a().onTrue(new IntakeDeploy(intakePivot));

    // Operator - Agitate Intake
    operator.b().onTrue(new AgitateIntake(intakePivot));

    // Operator - Stow Intake
    operator.x().onTrue(new IntakeStow(intakePivot));

    // Operator - Shoot Fuel
    operator
        .leftTrigger()
        .onTrue(new Shoot(shooter1, shooter2, shooter3, shooter4))
        .onFalse(new StopShooter(shooter1, shooter2, shooter3, shooter4));

    // Operator - Feed Fuel
    operator
        .rightTrigger()
        .whileTrue(new FeedFuel(feeder1, feeder2, hopper))
        .whileFalse(new StopFeederHopper(feeder1, feeder2, hopper));

    // Operator - Reverse Feeder
    operator
        .rightBumper()
        .whileTrue(new FeedFuelReverse(feeder1, feeder2, hopper))
        .whileFalse(new StopFeederHopper(feeder1, feeder2, hopper));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("StartIntake", new IntakeFuel(intake1, intake2));
    NamedCommands.registerCommand("StopIntake", new StopIntake(intake1, intake2));
    NamedCommands.registerCommand("AgitateIntake", new AgitateIntake(intakePivot));
    NamedCommands.registerCommand("DeployIntake", new IntakeDeploy(intakePivot));
    NamedCommands.registerCommand("StowIntake", new IntakeStow(intakePivot));
    NamedCommands.registerCommand(
        "AimAtHub",
        new LimelightAimCommand(drive, vision, () -> -driver.getLeftY(), () -> -driver.getLeftX()));
    NamedCommands.registerCommand(
        "ShootAndFeed",
        new ShootAndFeed(hopper, feeder1, feeder2, shooter1, shooter2, shooter3, shooter4));
    NamedCommands.registerCommand(
        "StopShootAndFeed",
        new StopShootAndFeed(hopper, feeder1, feeder2, shooter1, shooter2, shooter3, shooter4));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
