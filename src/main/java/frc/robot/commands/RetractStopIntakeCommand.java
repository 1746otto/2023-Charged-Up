package frc.robot.commands;

import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtensionSubsystem;

public class RetractStopIntakeCommand extends CommandBase {

  private final IntakeRollerSubsystem m_IntakeRollerSubsystem;
  private final IntakeExtensionSubsystem m_IntakeExtensionSubsystem;


  public RetractStopIntakeCommand(IntakeRollerSubsystem m_IntakeRollerSubsystem,
      IntakeExtensionSubsystem m_IntakeExtensionSubsystem) {
    this.m_IntakeRollerSubsystem = m_IntakeRollerSubsystem;
    this.m_IntakeExtensionSubsystem = m_IntakeExtensionSubsystem;

    addRequirements();
  }

  @Override
  public void initialize() {
    m_IntakeRollerSubsystem.setMotorStoppedSpeed();
    m_IntakeExtensionSubsystem.setMotorRetractionSpeed();

  }


}


