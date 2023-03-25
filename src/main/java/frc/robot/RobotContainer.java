package frc.robot;


import frc.robot.Autos.BalanceAuton;
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
import frc.robot.commands.basic.ElevatorRunUpCommand;
import frc.robot.commands.basic.NormalSpeedCommand;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BalancingCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

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
  private final JoystickButton operatorLeftJoystick =
      new JoystickButton(m_operator, XboxController.Button.kLeftStick.value);
  private final BooleanSupplier operatorRightTriggerSupplier = () -> {
    return Math.round(m_operator.getRightTriggerAxis()) > 0.0;
  };
  private final BooleanSupplier operatorLeftTriggerSupplier = () -> {
    return Math.round(m_operator.getLeftTriggerAxis()) > 0.0;
  };
  private final Trigger operatorRightTrigger = new Trigger(operatorRightTriggerSupplier);
  private final Trigger operatorLeftTrigger = new Trigger(operatorLeftTriggerSupplier);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();


  /* Commands */
  private final ScoringAlignCommand m_scoringAlignCommand = new ScoringAlignCommand(s_Swerve, true);
  private final Autos autos = new Autos(s_Swerve, m_VisionSubsystem, m_ElevatorSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    // Auton Selector
    m_chooser.setDefaultOption("scoringAlignCommand", m_scoringAlignCommand);
    m_chooser.addOption("Simple Auto", autos.exampleAuto());



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


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ----Driver Controls----

    driverStart.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    driverX.toggleOnTrue(new XLockCommand(s_Swerve));
    // driverLBumper.toggleOnTrue();
    // driverRBumper.toggleOnTrue();
    // driverRightTrigger.toggleOnTrue();
    // driverLeftTrigger.toggleOnTrue();

    // ----Operator Controls----

    // Elevator goes down to the origin position
    operatorA.onTrue(
        new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kOriginPosition)
            .withTimeout(1));
    // Elevator moves up to low position
    operatorB.onTrue(
        new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kLowPosition));
    // Elevator moves up to middle position
    operatorX.onTrue(
        new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kMidPosition));
    // Elevator moves up to high position
    operatorY.onTrue(
        new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kHighPosition));
    // Elevator runs down to beam break to get the zero position.
    operatorLeftBumper.onTrue(new ZeroOutElevatorCommand(m_ElevatorSubsystem));
  }


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
