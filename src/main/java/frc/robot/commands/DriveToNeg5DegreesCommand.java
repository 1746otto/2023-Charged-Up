package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class DriveToNeg5DegreesCommand extends CommandBase{
    private BooleanSupplier robotCentricSup;
    private Swerve s_Swerve;
    private double speed = 0.2;
    private  double currPitch;

    public DriveToNeg5DegreesCommand(Swerve s_Swerve, BooleanSupplier robotCentricSup){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.robotCentricSup = robotCentricSup;
        currPitch = -(s_Swerve.gyro.getRoll());
    }
    @Override
    public void execute(){
        s_Swerve.drive(new Translation2d(speed, 0).times(Constants.Swerve.maxSpeed),
        0.0, robotCentricSup.getAsBoolean(), true);
    }

    @Override 
    public boolean isFinished(){
        return ((s_Swerve.gyro.getRoll() < currPitch - 5));
    }
}