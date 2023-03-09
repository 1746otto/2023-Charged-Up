package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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
    Timer slewTimer;

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
        slewTimer = new Timer();

    }

    public boolean joystickBeingUsed(double rotationalVal) {
        if (rotationalVal == 0){
            return false;
        }
        return true;
    }

    @Override
    public void initialize() {
        slewTimer.start();
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
        double slowZone = 25.0;// Get current facing of robot and determine which direction is the fastest to target
        Translation2d driveVector = new Translation2d(translationVal, strafeVal).times(Math.sqrt(translationVal*translationVal+strafeVal*strafeVal)).times(Constants.Swerve.maxSpeed);
        double currentAngle = MathUtil.inputModulus(s_Swerve.getYaw().getDegrees(), 0, 360);
        System.out.println("Current Angle: " + currentAngle);

        if(joystickBeingUsed(rotationVal)) {
            rotationAngle = -600;
        }

        if(rotationAngle >= 0) {
            double clockwiseDist =  MathUtil.inputModulus(currentAngle - rotationAngle, 0, 360);
            System.out.println("Clock Dist: " + clockwiseDist);
            double counterClockwiseDist = 360 - clockwiseDist;//360 - MathUtil.inputModulus(currentAngle, 0, 360);
            System.out.println("CounterClock Dist: " + counterClockwiseDist);

            boolean moveClockwise = !(clockwiseDist <= counterClockwiseDist);

            // Set initial rotation velocity at max in desired direction of travel
            double travelDistance;
            if (moveClockwise) {
                travelDistance = counterClockwiseDist;
                rotationVal = .8;
            }
            else {
                travelDistance = -clockwiseDist;
                rotationVal = -.8;
            }

            // Slow down as you approach the target
            if (Math.abs(travelDistance) < slowZone) {
                rotationVal = Math.pow(((travelDistance * .6) / slowZone), 7);
            }
        }
        int targetMinusNum = 5;
        if (currentAngle == rotationAngle || (currentAngle > rotationAngle - targetMinusNum && currentAngle < rotationAngle + targetMinusNum) || ((rotationAngle == 0) && (currentAngle > 360 - targetMinusNum || currentAngle < rotationAngle + targetMinusNum))){
            rotationVal = 0;
        }
        
        
        // Slew limiting stuff
        Translation2d velocityVector = new Translation2d(
            Constants.Swerve.swerveKinematics.toChassisSpeeds(s_Swerve.getModuleStates()).vxMetersPerSecond, 
            Constants.Swerve.swerveKinematics.toChassisSpeeds(s_Swerve.getModuleStates()).vyMetersPerSecond
        );


        System.out.print("Robot Centric velocity: ");
        System.out.println(velocityVector);

        velocityVector.rotateBy(new Rotation2d(s_Swerve.gyro.getYaw()));
        
        double changeAngle = Math.atan2(driveVector.getY() - velocityVector.getY(), driveVector.getX() - velocityVector.getX());
        double magChange = Math.sqrt((driveVector.getX() - velocityVector.getX())*(driveVector.getX() - velocityVector.getX()) + (driveVector.getY() - velocityVector.getY())*(driveVector.getY() - velocityVector.getY()));
        System.out.print("Change Angle: ");
        System.out.println(changeAngle);
        System.out.print("Change Magnitude: ");
        System.out.println(magChange);
        if (magChange > Constants.Swerve.slewLimit)
            magChange = Constants.Swerve.slewLimit;
        slewTimer.reset();
        driveVector = new Translation2d(velocityVector.getX() + magChange*Math.cos(changeAngle), velocityVector.getY() + magChange*Math.sin(changeAngle));
        System.out.print("Drive Vector: ");
        System.out.println(driveVector);
        System.out.print("Velocity Vector: ");
        System.out.println(velocityVector);
        /* Drive */        
        s_Swerve.drive(
            driveVector, 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            false
        );
    }
} 