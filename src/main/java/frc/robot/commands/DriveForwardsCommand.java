package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class DriveForwardsCommand extends CommandBase{
    private Swerve s_Swerve;
    private double speed = Constants.Swerve.autonDriveSpeed;
    private BooleanSupplier robotCentricSup;
    private Timer time;

    public DriveForwardsCommand(Swerve s_Swerve, BooleanSupplier robotCentricSup){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void initialize(){
        time.start();
    }

    @Override
    public void execute(){
        s_Swerve.drive(new Translation2d(speed, 0).times(Constants.Swerve.maxSpeed),
        0.0, robotCentricSup.getAsBoolean(), true);
    }

    @Override
    public boolean isFinished(){
        return (time.hasElapsed(3.0));
    }
}