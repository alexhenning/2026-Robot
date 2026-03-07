//Apriltag detecting camera will be placed on shooter side to detect shooter tags and climber tags
package frc.robot.subsystems;

import java.time.format.TextStyle;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.Optional;
import java.util.Optional;
import java.util.Optional;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class VisionSS extends SubsystemBase{
    public PhotonCamera camera = new PhotonCamera("Camera_1");
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    public Optional<EstimatedRobotPose> robotPose;

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        PhotonPipelineResult result;
        if (!results.isEmpty()) {
            result = results.get(results.size() - 1);
        } else {
            return;
        }

        if (results.size() > 0) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            if (result.hasTargets()) {
                System.out.println("Has Target");
                robotPose = photonEstimator.estimateCoprocMultiTagPose(result)
                                .or(() -> photonEstimator.estimateLowestAmbiguityPose(result));
                System.out.println(robotPose);
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    System.out.println(target.getFiducialId());
                }
            } 

        } 
    }

    public Optional<EstimatedRobotPose> getRobotPose() {
        return robotPose;
    }
}
     