// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeRollerSubsystem;
// import frc.robot.subsystems.IntakeExtendSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeRollCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeRollerSubsystem m_RollSubsystem;
  // private final IntakeExtendSubsystem m_ExtendSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeRollCommand(IntakeRollerSubsystem subsystem) {
    m_RollSubsystem = subsystem;
    // m_ExtendSubsystem = subsystem2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    // addRequirements(subsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (m_ExtendSubsystem.isExtended()) {
        // delay
    
    m_RollSubsystem.runFullPower();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_RollSubsystem.runZeroPower();
  }

  // Returns true when the command should end.
  
}
