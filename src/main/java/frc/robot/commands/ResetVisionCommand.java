package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;

public class ResetVisionCommand extends CommandBase {
    VisionSubsystem m_visionSubsystem;
    Swerve m_swerve;
    Pose2d[] m_pose2ds;
    int m_accumulatorCount;
    double m_lastRot;
    public ResetVisionCommand(VisionSubsystem vision, Swerve swerve) {
        m_visionSubsystem = vision;
        m_swerve = swerve;
        
        addRequirements(m_visionSubsystem, m_swerve);
    }

    @Override
    public void initialize() {
        m_pose2ds = new Pose2d[VisionConstants.kFilterPasses];
        m_accumulatorCount = 0;
        m_lastRot = 0.0;
    }

    @Override
    public void execute() {
        if (m_visionSubsystem.isTargetValid()) {
            if (m_visionSubsystem.getPose2d().getRotation().getRadians() != m_lastRot) {
                m_pose2ds[m_accumulatorCount] = m_visionSubsystem.getPose2d();
                m_lastRot = m_pose2ds[m_accumulatorCount].getRotation().getRadians();
                m_accumulatorCount++;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_accumulatorCount >= VisionConstants.kFilterPasses;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted == true)
            return;
        else {
            double[] xs = new double[VisionConstants.kFilterPasses];
            double[] ys = new double[VisionConstants.kFilterPasses];
            double[] rots = new double[VisionConstants.kFilterPasses];
            
            for (int i = 0; i < VisionConstants.kFilterPasses; i++) {
                xs[i] = m_pose2ds[i].getX();
                ys[i] = m_pose2ds[i].getY();
                rots[i] = m_pose2ds[i].getRotation().getRadians();
            }

            Arrays.sort(xs);
            Arrays.sort(ys);
            Arrays.sort(rots);
            //VS Code thinks that this will never run because it is looking at the constant and evaluating it. 
            //We have changed the passes to 16 and it has detected that the logic is correct.
            //The bitwise operator >> must always be in parentheses because java has strange oder of operations.
            //We will leave the number of passes at 16 now because it makes the code easier to read. 
            if ((VisionConstants.kFilterPasses >> 2) * 4 == VisionConstants.kFilterPasses) {
                double xq1 = (xs[VisionConstants.kFilterPasses >> 2 - 1] + xs[VisionConstants.kFilterPasses >> 2])/2.0;
                double yq1 = (ys[VisionConstants.kFilterPasses >> 2 - 1] + ys[VisionConstants.kFilterPasses >> 2])/2.0;
                double rotq1 = (rots[VisionConstants.kFilterPasses >> 2 - 1] + rots[VisionConstants.kFilterPasses >> 2])/2.0;

                double xq3 = (xs[VisionConstants.kFilterPasses >> 1 - 1 + VisionConstants.kFilterPasses >> 2] + xs[VisionConstants.kFilterPasses >> 1 + VisionConstants.kFilterPasses >> 2])/2.0;
                double yq3 = (ys[VisionConstants.kFilterPasses >> 1 - 1 + VisionConstants.kFilterPasses >> 2] + ys[VisionConstants.kFilterPasses >> 1 + VisionConstants.kFilterPasses >> 2])/2.0;
                double rotq3 = (rots[VisionConstants.kFilterPasses >> 1 - 1 + VisionConstants.kFilterPasses >> 2] + rots[VisionConstants.kFilterPasses >> 1 + VisionConstants.kFilterPasses >> 2])/2.0;

                int xcount = 0;
                int ycount = 0;
                int rotcount = 0;

                double x = 0.0;
                double y = 0.0;
                double rot = 0.0;

                for (int i = 0; i < VisionConstants.kFilterPasses; i++) {
                    if ((xs[i] > xq1 - 1.5 * (xq3 - xq1)) && (xs[i] < xq3 + 1.5 * (xq3 - xq1))) {
                        x += xs[i];
                        xcount++;
                    }
                    if ((ys[i] > yq1 - 1.5 * (yq3 - yq1)) && (ys[i] < yq3 + 1.5 * (yq3 - yq1))) {
                        y += ys[i];
                        ycount++;
                    }
                    if ((rots[i] > rotq1 - 1.5 * (rotq3 - rotq1)) && (rots[i] < rotq3 + 1.5 * (rotq3 - rotq1))) {
                        rot += rots[i];
                        rotcount++;
                    }
                }

                x /= (double)xcount;
                y /= (double)ycount;
                rot /= (double)rotcount;

                m_swerve.swerveOdometry.resetPosition(Rotation2d.fromRadians(rot), m_swerve.getModulePositions(), new Pose2d(new Translation2d(x, y), Rotation2d.fromRadians(rot)));
 
            }
            else {
                int q1 = VisionConstants.kFilterPasses >> 2 - 1;
                int q3 = VisionConstants.kFilterPasses >> 1 + q1;

                int xcount = 0;
                int ycount = 0;
                int rotcount = 0;

                double x = 0.0;
                double y = 0.0;
                double rot = 0.0;

                for (int i = 0; i < VisionConstants.kFilterPasses; i++) {
                    if ((xs[i] > xs[q1] - 1.5 * (xs[q3] - xs[q1])) && (xs[i] < xs[q3] + 1.5 * (xs[q3] - xs[q1]))) {
                        x += xs[i];
                        xcount++;
                    }
                    if ((ys[i] > ys[q1] - 1.5 * (ys[q3] - ys[q1])) && (ys[i] < ys[q3] + 1.5 * (ys[q3] - ys[q1]))) {
                        y += ys[i];
                        ycount++;
                    }
                    if ((rots[i] > rots[q1] - 1.5 * (rots[q3] - rots[q1])) && (rots[i] < rots[q3] + 1.5 * (rots[q3] - rots[q1]))) {
                        rot += rots[i];
                        rotcount++;
                    }
                }

                x /= (double)xcount;
                y /= (double)ycount;
                rot /= (double)rotcount;

                m_swerve.swerveOdometry.resetPosition(Rotation2d.fromRadians(rot), m_swerve.getModulePositions(), new Pose2d(new Translation2d(x, y), Rotation2d.fromRadians(rot)));
            }
            
            
                        
        }
    }

}
