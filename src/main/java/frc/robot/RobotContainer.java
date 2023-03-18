package frc.robot;


import frc.robot.Autos.BalanceAuton;
import frc.robot.commands.AutomaticIntakeClamperCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.BalancingCommand;
import frc.robot.commands.DriveBackTo5DegreesCommand;
import frc.robot.commands.DriveForwardsCommand;
import frc.robot.commands.DriveOverChargeStationCommand;
import frc.robot.commands.DriveTo5DegreesCommand;
import frc.robot.commands.FourDimensionalBalancingCommand;
import frc.robot.commands.ScoringAlignCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.XLockCommand;
import frc.robot.commands.ZeroOutElevatorCommand;
import frc.robot.commands.basic.BalanceSpeedCommand;
import frc.robot.commands.basic.ClamperCloseCommand;
import frc.robot.commands.basic.ClamperOpenCommand;
import frc.robot.commands.basic.FlapCloseCommand;
import frc.robot.commands.basic.FlapOpenCommand;
import frc.robot.commands.basic.IndexerRollerIntakeCommand;
import frc.robot.commands.basic.ElevatorRunUpCommand;
import frc.robot.commands.basic.IndexerTreadIntakeCommand;
import frc.robot.commands.basic.IndexerTreadScoreCommand;
import frc.robot.commands.basic.IntakeExtensionExtendCommand;
import frc.robot.commands.basic.IntakeExtensionRetractCommand;
import frc.robot.commands.basic.IntakeRollerIntakeCommand;
import frc.robot.commands.basic.NormalSpeedCommand;
import frc.robot.commands.basic.PlungerExtendCommand;
import frc.robot.commands.basic.PlungerRetractCommand;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ElevatorRunToRequestCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.RetractStopIntakeCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /* Controllers */
  private final XboxController m_driver = new XboxController(ControllerConstants.kDriverPort);
  private final XboxController m_operator = new XboxController(ControllerConstants.kOperatorPort);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton driverStart =
      new JoystickButton(m_driver, XboxController.Button.kStart.value);

  private final JoystickButton driverY =
      new JoystickButton(m_driver, XboxController.Button.kY.value);
  private final JoystickButton driverB =
      new JoystickButton(m_driver, XboxController.Button.kB.value);
  private final JoystickButton driverA =
      new JoystickButton(m_driver, XboxController.Button.kA.value);
  private final JoystickButton driverX =
      new JoystickButton(m_driver, XboxController.Button.kX.value);
  private final JoystickButton driverLBumper =
      new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton driverRBumper =
      new JoystickButton(m_driver, XboxController.Button.kRightBumper.value);
  private final BooleanSupplier driverLTriggerSupplier = () -> {
    return 0 != Math.round(m_driver.getLeftTriggerAxis());
  };
  private final BooleanSupplier driverRTriggerSupplier = () -> {
    return 0 != Math.round(m_driver.getRightTriggerAxis());
  };
  public Trigger driverLeftTrigger = new Trigger(driverLTriggerSupplier);
  public Trigger driverRightTrigger = new Trigger(driverRTriggerSupplier);

  /* Operator Buttons */
  private final JoystickButton operatorStart =
      new JoystickButton(m_operator, XboxController.Button.kStart.value);
  private final JoystickButton operatorBack =
      new JoystickButton(m_operator, XboxController.Button.kBack.value);
  private final JoystickButton operatorY =
      new JoystickButton(m_operator, XboxController.Button.kY.value);
  private final JoystickButton operatorB =
      new JoystickButton(m_operator, XboxController.Button.kB.value);
  private final JoystickButton operatorA =
      new JoystickButton(m_operator, XboxController.Button.kA.value);
  private final JoystickButton operatorX =
      new JoystickButton(m_operator, XboxController.Button.kX.value);
  private final JoystickButton operatorLeftBumper =
      new JoystickButton(m_operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton operatorRightBumper =
      new JoystickButton(m_operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton operatorRightJoystick =
      new JoystickButton(m_operator, XboxController.Button.kRightStick.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final IntakeRollerSubsystem m_intakeRollerSubsystem = new IntakeRollerSubsystem();
  private final IntakeExtensionSubsystem m_intakeExtensionSubsystem =
      new IntakeExtensionSubsystem();
  private final IndexerTreadSubsystem m_indexerTreadSubsystem = new IndexerTreadSubsystem();
  private final IndexerRollerSubsystem m_indexerRollerSubsystem = new IndexerRollerSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final PlungerSubsystem m_plungerSubsystem = new PlungerSubsystem();
  private final ClamperSubsystem m_clamperSubsystem = new ClamperSubsystem();
  private final FlapSubsystem m_flapSubsystem = new FlapSubsystem();

  private final Compressor m_compressor =
      new Compressor(RobotConstants.kREVPH, PneumaticsModuleType.REVPH);

  /* Commands */
  private final ScoringAlignCommand m_scoringAlignCommand = new ScoringAlignCommand(s_Swerve, true);
  private final Autos autos = new Autos(s_Swerve, m_scoringAlignCommand,
      new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kMidPosition),
      new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kHighPosition),
      new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kOriginPosition),
      new FlapOpenCommand(m_flapSubsystem), new FlapCloseCommand(m_flapSubsystem),
      new PlungerExtendCommand(m_plungerSubsystem), new PlungerRetractCommand(m_plungerSubsystem),
      new ClamperOpenCommand(m_clamperSubsystem), new ClamperCloseCommand(m_clamperSubsystem),
      new DriveTo5DegreesCommand(s_Swerve), new DriveBackTo5DegreesCommand(s_Swerve),
      new BalancingCommand(s_Swerve), new DriveOverChargeStationCommand(s_Swerve),
      new DriveForwardsCommand(s_Swerve));

  private double elevatorSetPoint;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    disableCompressor();

    m_chooser.setDefaultOption("Auton1", "Auton1");
    m_chooser.addOption("Auton2", "Auton2");
    SmartDashboard.putData(m_chooser);
    disableCompressor();

    // SlewRateLimiter limiterT = new SlewRateLimiter(0.1, -0.1, 0);
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    s_Swerve.setDefaultCommand(
        /*
         * new TeleopSwerve( s_Swerve, () -> -driver.getRawAxis((int)
         * limiterT.calculate(translationAxis)), () ->
         * -driver.getRawAxis((int)limiterT.calculate(strafeAxis)), () ->
         * -driver.getRawAxis(rotationAxis), () -> false //robotCentric.getAsBoolean()
         * 
         * )
         */
        new TeleopSwerve(s_Swerve, () -> -m_driver.getRawAxis(translationAxis),
            () -> -m_driver.getRawAxis(strafeAxis), () -> -m_driver.getRawAxis(rotationAxis),
            DriverStation::getAlliance

        )

    );

    // This will be interupted if any of the subsystems below are being used, and will continue when
    // they aren't.
    // This allows us to not run this if we are actively running indexer, which will turn off when
    // beam is broken.
    // m_PlacerSubsystem.setDefaultCommand(new RunCommand(
    // // TODO: Tune these numbers to be realistic not guesses.
    // () -> {
    // if (m_ElevatorSubsystem.getElevatorEncoderValues() < ElevatorConstants.kOriginPosition
    // + 250
    // && m_ElevatorSubsystem.getElevatorEncoderValues() > ElevatorConstants.kOriginPosition
    // - 250
    // && m_IndexerSubsystem.beambreakBroken()) {
    // m_PlacerSubsystem.closeClamper();
    // }
    // }, m_PlacerSubsystem, m_ElevatorSubsystem, m_IndexerSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverStart.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    driverX.toggleOnTrue(new XLockCommand(s_Swerve)); // MAKE THIS XLOCK
    driverB.toggleOnTrue(new BalanceSpeedCommand());
    driverB.toggleOnFalse(new NormalSpeedCommand());
    driverLBumper.toggleOnTrue(new IntakeExtensionExtendCommand(m_intakeExtensionSubsystem));
    driverRBumper.toggleOnTrue(new IntakeExtensionRetractCommand(m_intakeExtensionSubsystem));
    driverRightTrigger.toggleOnTrue(new FlapOpenCommand(m_flapSubsystem)
        .andThen(new IntakeExtensionExtendCommand(m_intakeExtensionSubsystem)).andThen(
            new AutomaticIntakeClamperCommand(m_indexerRollerSubsystem, m_indexerTreadSubsystem,
                m_intakeRollerSubsystem, m_clamperSubsystem, m_intakeExtensionSubsystem)));
    driverLeftTrigger.toggleOnTrue(
        new RetractStopIntakeCommand(m_intakeRollerSubsystem, m_intakeExtensionSubsystem));



    // Elevator goes down to the origin position and then the flap closes
    operatorA.onTrue(new SequentialCommandGroup(new FlapOpenCommand(m_flapSubsystem),
        new PlungerRetractCommand(m_plungerSubsystem)));
    operatorA.onTrue(
        new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kOriginPosition));
    // Flap opens and then the elevator moves up to low position
    operatorB.onTrue(new SequentialCommandGroup(new FlapOpenCommand(m_flapSubsystem),
        new PlungerRetractCommand(m_plungerSubsystem)));
    operatorB.onTrue(
        new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kLowPosition));
    // Flap opens and then the elevator moves up to middle position
    operatorX.onTrue(new SequentialCommandGroup(new FlapOpenCommand(m_flapSubsystem),
        new PlungerRetractCommand(m_plungerSubsystem)));
    operatorX.onTrue(
        new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kMidPosition));
    // Flap opens and then the elevator moves up to high position
    operatorY.onTrue(new SequentialCommandGroup(new FlapOpenCommand(m_flapSubsystem),
        new PlungerRetractCommand(m_plungerSubsystem)));
    operatorY.onTrue(
        new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kHighPosition));
    // Elevator runs up and goes back down to beam break to get the zero position.
    operatorLeftBumper.onTrue(new SequentialCommandGroup(new FlapOpenCommand(m_flapSubsystem),
        new ZeroOutElevatorCommand(m_elevatorSubsystem), new FlapCloseCommand(m_flapSubsystem)));
    // Plunger extends and then opens the clamper
    operatorRightBumper.onTrue(new SequentialCommandGroup(
        new PlungerExtendCommand(m_plungerSubsystem), new ClamperOpenCommand(m_clamperSubsystem)));
    // // Intake and indexer run at the same time until the beam break is broken then the clamper
    // // closes(Used for elevator scoring).
    // operatorLeftBumper.onTrue(new SequentialCommandGroup(new FlapCloseCommand(m_flapSubsystem),
    // new ClamperOpenCommand(m_clamperSubsystem),
    // new ParallelDeadlineGroup(new IndexerTreadIntakeCommand(m_indexerTreadSubsystem),
    // new IntakeRollerIntakeCommand(m_intakeRollerSubsystem),
    // new IntakeExtensionExtendCommand(m_intakeExtensionSubsystem),
    // new IndexerRollerIntakeCommand(m_indexerRollerSubsystem)),
    // new ClamperCloseCommand(m_clamperSubsystem),
    // new IntakeExtensionRetractCommand(m_intakeExtensionSubsystem)));
    // // The flap opens and then the intake and indexer run at the same time(Used for low goals)
    // operatorBack.toggleOnTrue(new SequentialCommandGroup(new FlapOpenCommand(m_flapSubsystem),
    // new ClamperOpenCommand(m_clamperSubsystem),
    // new ParallelCommandGroup(new IndexerTreadScoreCommand(m_indexerTreadSubsystem),
    // new IntakeRollerIntakeCommand(m_intakeRollerSubsystem),
    // new IntakeExtensionExtendCommand(m_intakeExtensionSubsystem),
    // new IndexerRollerIntakeCommand(m_indexerRollerSubsystem))));
    // // Intake retracts and flap closes
    // operatorBack.toggleOnFalse(
    // new SequentialCommandGroup(new IntakeExtensionRetractCommand(m_intakeExtensionSubsystem),
    // new ClamperCloseCommand(m_clamperSubsystem), new FlapCloseCommand(m_flapSubsystem)));
  }

  public void enableCompressor() {
    m_compressor.enableDigital();
  }

  public void disableCompressor() {
    m_compressor.disable();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autos.scoreOne();
  }
}
