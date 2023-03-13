package frc.robot;


import frc.robot.commands.Autos;
import frc.robot.commands.ScoringAlignCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.basic.ClamperCloseCommand;
import frc.robot.commands.basic.ClamperOpenCommand;
import frc.robot.commands.basic.PlungerExtendCommand;
import frc.robot.commands.basic.PlungerRetractCommand;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ElevatorRunToRequestCommand;
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
  private final JoystickButton xBoxStart =
      new JoystickButton(m_driver, XboxController.Button.kStart.value);
  private final JoystickButton xBoxY = new JoystickButton(m_driver, XboxController.Button.kY.value);
  private final JoystickButton xBoxB = new JoystickButton(m_driver, XboxController.Button.kB.value);
  private final JoystickButton xBoxA = new JoystickButton(m_driver, XboxController.Button.kA.value);
  private final JoystickButton xBoxX = new JoystickButton(m_driver, XboxController.Button.kX.value);
  private final JoystickButton xBoxLBumper =
      new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value);

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

  private final Compressor m_compressor =
      new Compressor(RobotConstants.kREVPH, PneumaticsModuleType.REVPH);

  /* Commands */
  private final ScoringAlignCommand m_scoringAlignCommand = new ScoringAlignCommand(s_Swerve, true);
  private final Autos autos = new Autos(s_Swerve, m_scoringAlignCommand);

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
            () -> false, // robotCentric.getAsBoolean()
            () -> false, () -> false, () -> false, () -> false

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
    xBoxStart.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    xBoxA.onTrue(
        new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kOriginPosition));
    xBoxB.onTrue(
        new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kMidPosition));
    xBoxY.onTrue(
        new ElevatorRunToRequestCommand(m_elevatorSubsystem, ElevatorConstants.kHighPosition));
  }

  public void enableCompressor() {
    m_compressor.enableDigital();
  }

  public void disableCompressor() {
    m_compressor.disable();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autos.exampleAuto();
  }

}
