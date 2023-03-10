package frc.robot;

import com.fasterxml.jackson.databind.deser.std.ThrowableDeserializer;
import org.ejml.equation.IntegerSequence.Explicit;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.lang.Math;

import frc.robot.Autos.Auton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController m_controller = new XboxController(ControllerConstants.kport);
    private final XboxController m_controller2 = new XboxController(ControllerConstants.kport2);
  
  
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    

    // The robot's subsystems and commands are defined here...
  

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    private final JoystickButton faceUp = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton faceDown = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton faceRight = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton faceLeft = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    // private final Indexersubsystem m_IndexerSubsystem = new Indexersubsystem();
    private final PlacerSubsystem m_PlacerSubsystem = new PlacerSubsystem();
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private final Compressor m_Compressor = new Compressor(RobotConstants.kREVPH, PneumaticsModuleType.REVPH);
    private final Indexersubsystem m_IndexerSubsystem = new Indexersubsystem();

    


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
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
                () -> faceDown.getAsBoolean(),
                () -> faceRight.getAsBoolean(),
                () -> faceLeft.getAsBoolean()
                
            )
                
        );

        // Configure the button bindings
        configureButtonBindings();
    }


    private void configureDefaultCommands() {
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
                 () -> faceDown.getAsBoolean(),
                 () -> faceRight.getAsBoolean(),
                 () -> faceLeft.getAsBoolean()
                 
             )
                 
        );
        // This will be interupted if any of the subsystems below are being used, and will continue when they aren't.
        // This allows us to not run this if we are actively running indexer, which will turn off when beam is broken.
        m_PlacerSubsystem.setDefaultCommand(
            new RunCommand(
                //TODO: Tune these numbers to be realistic not guesses.
                () -> {
                    if (m_ElevatorSubsystem.getElevatorEncoderValues() < Constants.ElevatorConstants.kOriginPosition + 250 
                    && m_ElevatorSubsystem.getElevatorEncoderValues() > Constants.ElevatorConstants.kOriginPosition - 250
                    && m_IndexerSubsystem.beambreakBroken()) {
                        m_PlacerSubsystem.closeClamper();
                    }
                }, 
                m_PlacerSubsystem, 
                m_ElevatorSubsystem, 
                m_IndexerSubsystem
            )
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      JoystickButton xBoxY = new JoystickButton(m_controller, XboxController.Button.kY.value);
      JoystickButton xBoxB = new JoystickButton(m_controller, XboxController.Button.kB.value);
      JoystickButton xBoxA = new JoystickButton(m_controller, XboxController.Button.kA.value);
      JoystickButton xboxX = new JoystickButton(m_controller, XboxController.Button.kX.value);
      JoystickButton xBoxLBumper = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
      JoystickButton xBoxRBumper = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
  
  
  
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
       
    
        // xBoxLBumper.toggleOnTrue(new ClamperOpenCommand(m_PlacerSubsystem));
        // xBoxRBumper.toggleOnTrue(new PlungerExtendCommand(m_PlacerSubsystem));
        xBoxRBumper.onTrue(new SequentialCommandGroup(new IndexerCommand(m_IndexerSubsystem), new ClamperCloseCommand(m_PlacerSubsystem)));
        //xBoxLBumper.onTrue();
       

       // xBoxRBumper.toggleOnTrue(new PlungerExtendCommand(m_PlungerSubsystem, () -> m_ElevatorSubsystem.getElevatorEncoderValues()));
        //xBoxRBumper.toggleOnFalse(new SequentialCommandGroup(new PlungerRetractCommand(m_PlungerSubsystem), new ClamperOpenCommand(m_ClamperSubsystem)));
        // xBoxRBumper.toggleOnTrue(new SequentialCommandGroup(new PlungerRetractCommand(m_PlungerSubsystem), new ClamperOpenCommand(m_ClamperSubsystem)));
        // xBoxRBumper.toggleOnFalse(new SequentialCommandGroup(new PlungerExtendCommand(m_PlungerSubsystem), new ClamperCloseCommand(m_ClamperSubsystem)));
            
    
        JoystickButton xBoxY2 = new JoystickButton(m_controller2, XboxController.Button.kY.value);
        JoystickButton xBoxX2 = new JoystickButton(m_controller2, XboxController.Button.kX.value);
        JoystickButton xBoxA2 = new JoystickButton(m_controller2, XboxController.Button.kA.value);
       
    
        // xBoxA2.toggleOnTrue(new LowGoalCommand(m_IndexerSubsystem));
        // xBoxY2.toggleOnTrue(new IndexerCommand(m_IndexerSubsystem));
        // xBoxX2.toggleOnTrue(new IndexerReverseCommand(m_IndexerSubsystem));
    }
    public void enableCompressor(){
        m_Compressor.enableDigital();
    }
    public void disableCompressor(){
        m_Compressor.disable();
    }

 

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new Auton(s_Swerve);
    }
}

