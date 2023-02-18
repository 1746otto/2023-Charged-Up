// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.IntakeRollCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_controller = new XboxController(ControllerConstants.kport);

  // The robot's subsystems and commands are defined here...

  private final IntakeExtendSubsystem m_IntakeExtendSubsystem = new IntakeExtendSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeRollerSubsystem m_IntakeRollerSubsystem = new IntakeRollerSubsystem();
  

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
    JoystickButton xBoxY = new JoystickButton(m_controller, XboxController.Button.kY.value);
    JoystickButton xBoxB = new JoystickButton(m_controller, XboxController.Button.kB.value);
    JoystickButton xBoxA = new JoystickButton(m_controller, XboxController.Button.kA.value);
    JoystickButton xboxX = new JoystickButton(m_controller, XboxController.Button.kX.value);
    JoystickButton xBoxLBumper =
        new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    JoystickButton xBoxRBumper =
        new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);

       

        xboxX.toggleOnTrue(new IntakeExtendCommand(m_IntakeExtendSubsystem).alongWith(new IntakeRollCommand(m_IntakeRollerSubsystem, m_IntakeExtendSubsystem)));
        xboxX.toggleOnFalse(new IntakeRetractCommand(m_IntakeExtendSubsystem));


    
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
  
}
