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
import frc.robot.subsystems.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
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
  
}
