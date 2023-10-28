package frc.robot;

import frc.robot.Autos.BalanceAuton;
import frc.robot.commands.ArmHomeCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.BalancingCommand;
import frc.robot.commands.DriveBackTo5DegreesCommand;
import frc.robot.commands.DriveForwardsCommand;
import frc.robot.commands.DriveOverChargeStationCommand;
import frc.robot.commands.DriveTo5DegreesCommand;
import frc.robot.commands.FourDimensionalBalancingCommand;
import frc.robot.commands.OuttakingSequentialCommand;
import frc.robot.commands.ScoringAlignCommand;
import frc.robot.commands.ShootCubeHighCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.UpdateOdometryCommand;
import frc.robot.commands.XLockCommand;
import frc.robot.commands.ZeroOutElevatorCommand;
import frc.robot.commands.basic.ArmPositionStopCommand;
import frc.robot.commands.basic.ArmRollerIntakeCommand;
import frc.robot.commands.basic.ArmRollerOuttakeCommand;
import frc.robot.commands.basic.ArmRollerRunInCommand;
import frc.robot.commands.basic.ArmRollerShootCommand;
import frc.robot.commands.basic.ArmRollerStopCommand;
import frc.robot.commands.basic.ArmToSmartDashboardCommand;
import frc.robot.commands.basic.ArmRequestSelectorCommand;
import frc.robot.commands.basic.BalanceSpeedCommand;
import frc.robot.commands.basic.CatapultRunToMax;
import frc.robot.commands.basic.CatapultRunToMin;
import frc.robot.commands.basic.LedConeCommand;
import frc.robot.commands.basic.LedCubeCommand;
import frc.robot.commands.basic.NormalSpeedCommand;
import frc.robot.commands.basic.TheDunkCommand;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.commands.ElevatorRequestSelectorCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BalancingCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootCubeHighCommand;
import frc.robot.commands.ConeIntakeCommand;
import frc.robot.commands.CatapultAutonCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final SendableChooser<Command> m_chooser = new SendableChooser<>();

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
  private final JoystickButton L3 =
      new JoystickButton(m_operator, XboxController.Button.kLeftStick.value);

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final ArmPositionSubsystem m_ArmPosSubystem = new ArmPositionSubsystem();
  private final ArmRollersSubsystem m_ArmRollersSubsystem = new ArmRollersSubsystem();
  private final ConeDunkerSubsytem m_ConeDunkerSubsytem = new ConeDunkerSubsytem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  private final CatapultSubsystem m_CatapultSubsystem = new CatapultSubsystem();
  private final Vision vision =
      new Vision(s_Swerve.getYaw()::getDegrees, s_Swerve::addVisionMeasurement);

  /* Commands */
  private final ScoringAlignCommand m_scoringAlignCommand = new ScoringAlignCommand(s_Swerve, true);
  private final Autos autos = new Autos(s_Swerve, m_VisionSubsystem, m_ElevatorSubsystem,
      m_ArmPosSubystem, m_ArmRollersSubsystem, m_ConeDunkerSubsytem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Vision.getLastResult();

    // Auton Selector
    /*
     * m_chooser.setDefaultOption("Score only", autos.scoreOne());
     * m_chooser.addOption("3 piece bump", autos.threePieceBumpCatapult());
     * m_chooser.addOption("Score,community,balance ", autos.balanceAfterCharge());
     * SmartDashboard.putData("Auton Selector: ", m_chooser);
     */


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

    // m_VisionSubsystem.setDefaultCommand(
    // new UpdateOdometryCommand(m_VisionSubsystem, s_Swerve.getYaw()::getDegrees,
    // s_Swerve::getPose, s_Swerve.poseEstimator::addVisionMeasurement));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ----Driver Controls----
    // Pushing

    // Cube intaking
    // cubes off ground
    // TEST NOW: INTAKE POSITION MIGHT BE WRONG
    driverRightTrigger.onTrue(new SequentialCommandGroup(
        new ParallelDeadlineGroup(new ArmRollerIntakeCommand(m_ArmRollersSubsystem),
            new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmIntakeAndScorePos),
            new ElevatorRequestSelectorCommand(m_ElevatorSubsystem,
                ElevatorConstants.kNewCubeIntakePos)),
        new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmRestPos),
        new ElevatorRequestSelectorCommand(m_ElevatorSubsystem,
            ElevatorConstants.kOriginPosition)));

    // Cone intaking
    driverRightBumper.onTrue(new SequentialCommandGroup(
        new ElevatorRequestSelectorCommand(m_ElevatorSubsystem,
            ElevatorConstants.kConeElevatorIntakePos),
        new ParallelDeadlineGroup(new ArmRollerIntakeCommand(m_ArmRollersSubsystem),
            new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmConeIntakePos)),
        new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmRestPos),
        new ElevatorRequestSelectorCommand(m_ElevatorSubsystem,
            ElevatorConstants.kOriginPosition)));


    // driverLeftBumper.onTrue(new SequentialCommandGroup(new ParallelDeadlineGroup(
    // new ArmRollerIntakeCommand(m_ArmRollersSubsystem),
    // new ElevatorRequestSelectorCommand(m_ElevatorSubsystem, ElevatorConstants.kCubeIntakePos),
    // new WaitCommand(0.4)
    // .until(() -> m_ElevatorSubsystem.isElevatorAtReq(ElevatorConstants.kCubeIntakePos)),
    // new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmIntakeAndScorePos))));

    // driverLeftBumper.onTrue(
    // new ElevatorRequestSelectorCommand(m_ElevatorSubsystem, ElevatorConstants.kMidPosition));
    // driverLeftBumper.onFalse(
    // new ElevatorRequestSelectorCommand(m_ElevatorSubsystem, ElevatorConstants.kOriginPosition));
    // OLD LEFT BUMPER

    // humman player station intake
    driverLeftTrigger
        .onTrue(new ParallelDeadlineGroup(new ArmRollerIntakeCommand(m_ArmRollersSubsystem),
            new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.ksubstationPosition)));


    driverA.whileTrue(new ArmRollerOuttakeCommand(m_ArmRollersSubsystem));

    driverB.onTrue(new SequentialCommandGroup(
        new ElevatorRequestSelectorCommand(m_ElevatorSubsystem, ElevatorConstants.kMidPosition),
        new WaitCommand(0.8).until(() -> {
          return m_ElevatorSubsystem.getPosition() >= ElevatorConstants.kMidPosition - 4;
        }), new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmMidPos)));

    driverX.onTrue(new SequentialCommandGroup(
        new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmRestPos),
        new WaitCommand(0.5).until(() -> m_ArmPosSubystem.armAtReq(ArmConstants.kArmRestPos)),
        new ElevatorRequestSelectorCommand(m_ElevatorSubsystem,
            ElevatorConstants.kOriginPosition)));

    driverY.onTrue(new SequentialCommandGroup(
        new ElevatorRequestSelectorCommand(m_ElevatorSubsystem, ElevatorConstants.kHighPosition),
        new WaitCommand(1.2)
            .until(() -> m_ElevatorSubsystem.getPosition() >= ElevatorConstants.kHighPosition - 4),
        new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmHighScoringPos)));

    operatorX.whileTrue(new XLockCommand(s_Swerve));
    driverStart.onTrue(new InstantCommand(() -> {
      s_Swerve.gyro.setYaw((DriverStation.getAlliance() == Alliance.Red) ? 180 : 0);
    }));
    // bowling piece to mid
    driverLeftBumper
        .onTrue(new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmMidShootPos));

    operatorRightBumper.whileTrue(new BalanceSpeedCommand());

    operatorStart.onTrue(new InstantCommand(() -> {
      s_Swerve.resetModulesToAbsolute();
    }).andThen(new WaitCommand(.5)));
    operatorLeftBumper.whileTrue(new ArmHomeCommand(m_ArmPosSubystem));
    // Elevator runs down to beam break to get the zero position.
    // operatorA.onTrue(new ZeroOutElevatorCommand(m_ElevatorSubsystem));

    /*
     * operatorLeftTrigger .onTrue(new SequentialCommandGroup(new
     * ArmRollerStopCommand(m_ArmRollersSubsystem), new ArmRequestSelectorCommand(m_ArmPosSubystem,
     * ArmConstants.kArmRestPos)));
     */
    operatorY.onTrue(
        new ShootCubeHighCommand(m_ElevatorSubsystem, m_ArmPosSubystem, m_ArmRollersSubsystem));
    operatorA.whileTrue(new ParallelCommandGroup(
        new ElevatorRequestSelectorCommand(m_ElevatorSubsystem,
            ElevatorConstants.kConeElevatorIntakePos),
        new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmConeIntakePos),
        new ArmRollerIntakeCommand(m_ArmRollersSubsystem)));
    operatorA.onFalse(new ParallelCommandGroup(
        // new ElevatorRequestSelectorCommand(m_ElevatorSubsystem,
        // ElevatorConstants.kOriginPosition),
        new ArmRequestSelectorCommand(m_ArmPosSubystem, ArmConstants.kArmRestPos),
        new InstantCommand(() -> m_ArmRollersSubsystem.armRollerStow(), m_ArmRollersSubsystem),
        new ElevatorRequestSelectorCommand(m_ElevatorSubsystem,
            ElevatorConstants.kOriginPosition)));


    L3.toggleOnTrue(new RepeatCommand(new InstantCommand(() -> m_LedSubsystem
        .setToHue((int) ((Math.atan2(m_operator.getRawAxis(XboxController.Axis.kLeftX.value),
            m_operator.getRawAxis(XboxController.Axis.kLeftY.value)) + Math.PI) * 90 / Math.PI))))
                .finallyDo((boolean interrupted) -> m_LedSubsystem.setLedOff()));
    operatorRightTrigger.whileTrue(new LedConeCommand(m_LedSubsystem));
    operatorLeftTrigger.whileTrue(new LedCubeCommand(m_LedSubsystem));
  }


  public Command getAutonomousCommand() {
    // An Exammple Command will run in autonomous
    // balance, BL, B2 for practice field
    // return autos.balanceAfterCharge();
    return autos.FinalBruh();
  }


}

