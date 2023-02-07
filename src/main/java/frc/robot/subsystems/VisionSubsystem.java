package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    PhotonPipelineResult pipelineResult;
    PhotonTrackedTarget target;
    public VisionSubsystem() {
        camera = new PhotonCamera("OV5647");
    }
    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            target = result.getBestTarget();
            System.out.println(target.getYaw());
        }
        else {
            System.out.println("i don't see nuthin");
        }
    }
}
