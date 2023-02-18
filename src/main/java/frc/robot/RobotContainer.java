package frc.robot;

<<<<<<< HEAD
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
import frc.robot.subsystems.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.*;
=======
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.lang.Math;

import frc.robot.Autos.Auton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
>>>>>>> origin/main

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
<<<<<<< HEAD
  private final XboxController m_controller = new XboxController(ControllerConstants.kport);
  private final XboxController m_controller2 = new XboxController(ControllerConstants.kport2);


  // The robot's subsystems and commands are defined here...
  private final Indexersubsystem m_IndexerSubsystem = new Indexersubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton xBoxY2 = new JoystickButton(m_controller2, XboxController.Button.kY.value);
    JoystickButton xBoxX2 = new JoystickButton(m_controller2, XboxController.Button.kX.value);
    JoystickButton xBoxA2 = new JoystickButton(m_controller2, XboxController.Button.kA.value);
   

    xBoxA2.toggleOnTrue(new LowGoalCommand(m_IndexerSubsystem));
    xBoxY2.toggleOnTrue(new IndexerCommand(m_IndexerSubsystem));
    xBoxX2.toggleOnTrue(new IndexerReverseCommand(m_IndexerSubsystem));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }


/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
=======
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
    private final JoystickButton faceRight = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton faceLeft = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
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

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new Auton(s_Swerve);
    }
>>>>>>> origin/main
}

