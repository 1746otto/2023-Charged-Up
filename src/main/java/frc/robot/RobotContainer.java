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
import frc.robot.commands.basic.ArmPositionStopCommand;
import frc.robot.commands.basic.ArmRollerIntakeCommand;
import frc.robot.commands.basic.ArmRollerOuttakeCommand;
import frc.robot.commands.basic.ArmRollerStopCommand;
import frc.robot.commands.basic.ArmRunToRequestCommand;
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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ArmConstants;
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
  private final JoystickButton driverBack =
      new JoystickButton(m_driver, XboxController.Button.kBack.value);
  private final JoystickButton driverY =
      new JoystickButton(m_driver, XboxController.Button.kY.value);
  private final JoystickButton driverB =
      new JoystickButton(m_driver, XboxController.Button.kB.value);
  private final JoystickButton driverA =
      new JoystickButton(m_driver, XboxController.Button.kA.value);
  private final JoystickButton driverX =
      new JoystickButton(m_driver, XboxController.Button.kX.value);
  private final JoystickButton driverLeftBumper =
      new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton driverRightBumper =
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
  private final ArmPositionSubsystem m_ArmPosSubystem = new ArmPositionSubsystem();
  private final ArmRollersSubsystem m_ArmRollersSubsystem = new ArmRollersSubsystem();

  private final Compressor m_compressor =
      new Compressor(RobotConstants.kREVPH, PneumaticsModuleType.REVPH);

  /* Commands */
  private final ScoringAlignCommand m_scoringAlignCommand = new ScoringAlignCommand(s_Swerve, true);
  private final Autos autos = new Autos(s_Swerve, m_VisionSubsystem, m_ElevatorSubsystem);

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
            DriverStation::getAlliance));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ----Driver Controls----

    // Elevator goes down to the origin position
    driverA.whileTrue(new ArmRollerOuttakeCommand(m_ArmRollersSubsystem));
    driverA.whileFalse(new ArmRollerStopCommand(m_ArmRollersSubsystem));
    // Elevator moves up to mid position
    driverB.onTrue(new SequentialCommandGroup(new WaitCommand(0.3),
        new ArmRunToRequestCommand(m_ArmPosSubystem, ArmConstants.kArmIntakeAndScorePos)));
    driverB.onTrue(
        new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kMidPosition));
    // Elevator moves up to origin position
    driverX.onTrue(new ParallelCommandGroup(
        new ArmRunToRequestCommand(m_ArmPosSubystem, ArmConstants.kArmRestPos),
        new SequentialCommandGroup(new WaitCommand(0.4), new ElevatorRunToRequestCommand(
            m_ElevatorSubsystem, ElevatorConstants.kOriginPosition))));
    // Elevator moves up to high position
    driverY.onTrue(new SequentialCommandGroup(new WaitCommand(0.3),
        new ArmRunToRequestCommand(m_ArmPosSubystem, ArmConstants.kArmIntakeAndScorePos)));
    driverY.onTrue(
        new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kHighPosition));
    // Arm rollers intake or outtake
    driverRightBumper.toggleOnTrue(new ArmRollerIntakeCommand(m_ArmRollersSubsystem));
    driverRightBumper.toggleOnFalse(new ArmRollerStopCommand(m_ArmRollersSubsystem));
    // Robots x locks for balancing
    driverBack.onTrue(new XLockCommand(s_Swerve));
    // Arm goes to score and intake position
    driverLeftBumper
        .onTrue(new ArmRunToRequestCommand(m_ArmPosSubystem, ArmConstants.kArmIntakeAndScorePos));

    driverLeftTrigger.toggleOnTrue(new ParallelCommandGroup(
        new ArmRunToRequestCommand(m_ArmPosSubystem, ArmConstants.kArmIntakeAndScorePos),
        new ArmRollerIntakeCommand(m_ArmRollersSubsystem)));
    driverLeftTrigger.toggleOnTrue(
        new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kConeIntakePos));

    driverLeftTrigger.toggleOnFalse(new ParallelCommandGroup(
        new ArmRunToRequestCommand(m_ArmPosSubystem, ArmConstants.kArmRestPos),
        new ArmRollerIntakeCommand(m_ArmRollersSubsystem)));
    driverLeftTrigger.toggleOnFalse(
        new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kOriginPosition));



    // Elevator runs down to beam break to get the zero position.
    operatorStart.onTrue(new ZeroOutElevatorCommand(m_ElevatorSubsystem));
  }

  public void enableCompressor() {
    m_compressor.enableDigital();
  }

  public void disableCompressor() {
    m_compressor.disable();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autos.scoreOneBalance();
  }
}
