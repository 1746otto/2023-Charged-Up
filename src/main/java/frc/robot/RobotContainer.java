package frc.robot;


import frc.robot.commands.Autos;
import frc.robot.commands.ClamperCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IndexerReverseCommand;
import frc.robot.commands.ResetVisionCommand;
import frc.robot.commands.ScoringAlignCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.IntakeRollCommand;
import frc.robot.commands.LowGoalCommand;
import frc.robot.subsystems.ClamperSubsystem;
import frc.robot.subsystems.Flapsubsystem;
import frc.robot.subsystems.Indexersubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorRunToRequestCommand;
import java.lang.Math;


import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController m_controller = new XboxController(ControllerConstants.kport);
    private final XboxController m_controller2 = new XboxController(ControllerConstants.kport2);
    private final Alliance allianceColor = DriverStation.getAlliance();
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    private final JoystickButton faceUp = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton faceDown = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton faceRight = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton faceLeft = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton balance = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /*Commands */
    private final ScoringAlignCommand m_scoringAlignCommand = new ScoringAlignCommand(s_Swerve, true);
    private final Autos autos = new Autos(s_Swerve, m_scoringAlignCommand);
    private final Indexersubsystem m_IndexerSubsystem = new Indexersubsystem();
    private final Flapsubsystem m_Flapsubsystem = new Flapsubsystem();
    private final IntakeExtendSubsystem m_IntakeExtendSubsystem = new IntakeExtendSubsystem();
    private final Compressor m_compressor = new Compressor(RobotConstants.kREVPH, PneumaticsModuleType.REVPH);
    final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private final ClamperSubsystem m_ClamperSubsystem = new ClamperSubsystem();

    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
     
        m_chooser.setDefaultOption("Auton1", "Auton1");
        //m_chooser.addOption("Auton2", getAutonomousCommand());
        m_chooser.addOption("Auton2", "Auton2");
        SmartDashboard.putData(m_chooser);
        disableCompressor();
    
       // SlewRateLimiter limiterT = new SlewRateLimiter(0.1, -0.1, 0);
        s_Swerve.setDefaultCommand(
           /*  new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis((int) limiterT.calculate(translationAxis)), 
                () -> -driver.getRawAxis((int)limiterT.calculate(strafeAxis)), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false   //robotCentric.getAsBoolean()
                
            )*/
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,   //robotCentric.getAsBoolean()
                () -> faceUp.getAsBoolean(),
                () -> faceUp.getAsBoolean(),
                () -> faceUp.getAsBoolean(),
                () -> faceUp.getAsBoolean()
                
            )
                
        );

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
  
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        JoystickButton xBoxYButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
        JoystickButton xBoxBButton = new JoystickButton(m_controller, XboxController.Button.kB.value);
        JoystickButton xBoxAButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
        JoystickButton xBoxXButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
        JoystickButton xBoxStartButton = new JoystickButton(m_controller, XboxController.Button.kStart.value);
        JoystickButton xBoxLBumperButton =
            new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
        JoystickButton xBoxRBumperButton =
            new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    
            xBoxLBumperButton.toggleOnTrue(new ClamperCommand(m_ClamperSubsystem));
            
    
    
        JoystickButton xBoxX2 = new JoystickButton(m_controller2, XboxController.Button.kX.value);
        JoystickButton xBoxA2 = new JoystickButton(m_controller2, XboxController.Button.kA.value);
        JoystickButton xBoxY2 = new JoystickButton(m_controller2, XboxController.Button.kY.value);
        JoystickButton xBoxRBumper = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    
       
    
        xBoxA2.toggleOnTrue(new LowGoalCommand(m_IndexerSubsystem, m_Flapsubsystem));
        xBoxY2.toggleOnTrue(new IndexerCommand(m_IndexerSubsystem));
        xBoxX2.toggleOnTrue(new IndexerReverseCommand(m_IndexerSubsystem));
        xBoxBButton.toggleOnTrue(new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kHighPosition));
        xBoxXButton.toggleOnTrue(new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kMidPosition));
        xBoxAButton.toggleOnTrue(new ElevatorRunToRequestCommand(m_ElevatorSubsystem, ElevatorConstants.kOriginPosition));
    }

    public void enableCompressor() {
        m_compressor.enableDigital();
    }

    public void disableCompressor() {
        m_compressor.disable();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autos.exampleAuto();
    }
  
}
