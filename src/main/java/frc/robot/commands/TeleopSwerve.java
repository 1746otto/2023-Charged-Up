package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier faceUpSup;
    private BooleanSupplier faceDownSup;
    private BooleanSupplier faceRightSup;
    private BooleanSupplier faceLeftSup;
    private int rotationAngle; 
    private double prevError;

    private double kP = 0.015;
    private double kD = 1;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier faceUpSup, BooleanSupplier faceDownSup, BooleanSupplier faceRightSup, BooleanSupplier faceLeftSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.faceUpSup = faceUpSup;
        this.faceDownSup = faceDownSup;
        this.faceLeftSup = faceLeftSup;
        this.faceRightSup = faceRightSup;
        rotationAngle = -600;
        prevError = 0;

    }

    public boolean joystickBeingUsed(double rotationalVal) {
        if (rotationalVal == 0){
            return false;
        }
        return true;
    }

    @Override
    public void execute() {
        // Turning Buttons
        if(faceUpSup.getAsBoolean()) {
            rotationAngle = 0;
        }
        if(faceDownSup.getAsBoolean()) {
            rotationAngle = 180;
        }
        if(faceRightSup.getAsBoolean()) {
            rotationAngle = 270;
        }
        if(faceLeftSup.getAsBoolean()) {
            rotationAngle = 90;
        }
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        //translationVal = Math.copySign(translationVal*translationVal, translationSup.getAsDouble());
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        //translationVal = Math.copySign(translationVal*strafeVal, strafeSup.getAsDouble());
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double currentAngle = MathUtil.inputModulus(s_Swerve.getYaw().getDegrees(), 0, 360);
        //System.out.println("Current Angle: " + currentAngle);

        if(joystickBeingUsed(rotationVal)) {
            rotationAngle = -600;
            prevError = 0;
        }

        if(rotationAngle >= 0) {
            double clockwiseDist =  MathUtil.inputModulus(currentAngle - rotationAngle, 0, 360);
            //System.out.println("Clock Dist: " + clockwiseDist);
            double counterClockwiseDist = 360 - clockwiseDist;//360 - MathUtil.inputModulus(currentAngle, 0, 360);
            //System.out.println("CounterClock Dist: " + counterClockwiseDist);

            boolean moveCounterClockwise = !(clockwiseDist <= counterClockwiseDist);

            // Set initial rotation velocity at max in desired direction of travel
            double error;
            if (moveCounterClockwise) {
                error = counterClockwiseDist;
                rotationVal = .8;
            }
            else {
                error = -clockwiseDist;
                rotationVal = -.8;
            }

            rotationVal = kP * error;
            // rotationVal = (error - prevError) * kD + (kP * error);

            if (rotationVal > 1){
                rotationVal = 1;
            }else if (rotationVal < -1){
                rotationVal = -1;
            }
            
            prevError = error;
        }
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal,strafeVal).times(Math.sqrt(translationVal*translationVal + strafeVal*strafeVal)).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
} 