package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import java.io.Console;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // TurretSubsystem turretSubsystem;
  // IndexerSubsystem indexerSubsystem;
  Vision vision;
  private final XboxController controller = new XboxController(0);
  private final JoystickButton button =
      new JoystickButton(controller, XboxController.Button.kY.value);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // turretSubsystem = new TurretSubsystem();
    vision = new Vision();


  }

  private void configureDefaultCommands() {}


  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(null);
  }


}

