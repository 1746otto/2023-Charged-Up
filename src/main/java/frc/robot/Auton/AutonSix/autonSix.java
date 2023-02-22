package frc.robot.Auton.AutonSix;


import frc.robot.commands.LowGoalCommand;
import frc.robot.subsystems.Indexersubsystem;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class autonSix extends SequentialCommandGroup{
    public autonSix(Swerve s_Swerve){
        addCommands(new SequentialCommandGroup(new LowGoalCommand(new Indexersubsystem())));
        // Add balenceing Command Here
    }
    
}
