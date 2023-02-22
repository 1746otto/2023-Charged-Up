package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UnbalancingCommand extends CommandBase{
    private Swerve s_Swerve;
    private BooleanSupplier robotCentricSup;
    private double xError;
    private double xPrevError;
    private double yError;
    private double yPrevError;
    private double kP = 0.01;
    private double kD = 0.1;
    private double speed;
    private double currRoll;

    public UnbalancingCommand(Swerve s_Swerve, BooleanSupplier robotCentricSup){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.robotCentricSup = robotCentricSup;
        this.currRoll = -(s_Swerve.gyro.getRoll());
    }

    @Override
    public void execute() {
        xError = s_Swerve.gyro.getRoll();
        System.out.println("Roll: " + xError);

        speed = kP * xError;
        // xSpeed = (xError - xPrevError) * kD + (kP * xError);
        
        if (speed > Constants.Swerve.autonDriveSpeed){
            speed = Constants.Swerve.autonDriveSpeed;
        }else if (speed < -Constants.Swerve.autonDriveSpeed){
            speed = -Constants.Swerve.autonDriveSpeed;
        }

        xPrevError = xError;
        s_Swerve.drive(new Translation2d(speed, 0).times(Constants.Swerve.maxSpeed),
        0.0, robotCentricSup.getAsBoolean(), true);
    }

    @Override 
    public boolean isFinished(){
        return (s_Swerve.gyro.getRoll() > currRoll + 5);
    }
}