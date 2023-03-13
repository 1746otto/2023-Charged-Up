package frc.robot.commands.basic;

import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeRollerStopCommand extends CommandBase {
  private final IntakeRollerSubsystem m_intakeRollerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeRollerStopCommand(IntakeRollerSubsystem subsystem) {
    m_intakeRollerSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeRollerSubsystem.setMotorStoppedSpeed();
  }
}
