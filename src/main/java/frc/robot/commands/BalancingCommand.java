package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalancingCommand extends CommandBase{
    private Swerve s_Swerve;
    private BooleanSupplier robotCentricSup;
    private double xError;
    private double xPrevError;
    private double yError;
    private double yPrevError;
    private double kP = 0.01;
    private double kD = 0.1;
    private double speed;

    public BalancingCommand(Swerve s_Swerve, BooleanSupplier robotCentricSup){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        if (s_Swerve.gyro.getRoll() != 0){
        xError = -(s_Swerve.gyro.getRoll());
        System.out.println("Roll: " + xError);

        speed = kP * xError;
        // xSpeed = (xError - xPrevError) * kD + (kP * xError);

        if (speed > .2){
            speed = .2;
        }else if (speed < -.2){
            speed = -.2;
        }

        xPrevError = xError;
        }

        // if (s_Swerve.gyro.getPitch() != 0){
        //     yError = -(s_Swerve.gyro.getPitch());

        //     speed = kP * yError;
        //     // ySpeed = (yError - yPrevError) * kD + (kP * yError);

        //     if (speed > .2){
        //         speed = .2;
        //     }else if (speed < -.2){
        //         speed = -.2;
        //     }

        //     yPrevError = yError;
        // }

        s_Swerve.drive(new Translation2d(speed, 0).times(Constants.Swerve.maxSpeed),
        0.0, robotCentricSup.getAsBoolean(), true);
    }

    @Override
    public void end(boolean interrupted){
        s_Swerve.drive(new Translation2d(0, 0),
        180.0, robotCentricSup.getAsBoolean(), true);
    }

    @Override 
    public boolean isFinished(){
        return (s_Swerve.gyro.getRoll() == 0);
    }
}