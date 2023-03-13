package frc.robot.commands.basic;

import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeRollerSetSpeedCommand extends CommandBase {
  private final IntakeRollerSubsystem m_intakeRollerSubsystem;
  private final double m_speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeRollerSetSpeedCommand(IntakeRollerSubsystem subsystem, double speed) {
    m_intakeRollerSubsystem = subsystem;
    addRequirements(subsystem);

    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeRollerSubsystem.setMotorSpeed(m_speed);
  }
}
