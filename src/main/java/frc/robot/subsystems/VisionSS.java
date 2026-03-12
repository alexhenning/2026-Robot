//Apriltag detecting camera will be placed on shooter side to detect shooter tags and climber tags
package frc.robot.subsystems;
import frc.robot.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSS extends SubsystemBase{
    public PhotonCamera camera = new PhotonCamera("ShooterCam");
    public PhotonCamera driveCamera = new PhotonCamera("DriverCam");
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.2286, -0.3175, 0.127), new Rotation3d(0, 0, 0));
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    public Optional<EstimatedRobotPose> robotPose;
    public double distanceToHub;
    boolean isblue;
    boolean targetVisible = false;

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        PhotonPipelineResult result;
        if (!results.isEmpty()) {
            result = results.get(results.size() - 1);
        } else {
            boolean targetVisible = false;
            SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
            return;
        }

        if (results.size() > 0) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            if (result.hasTargets()) {
                boolean targetVisible = true;
                SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
                // System.out.println("Has Target");
                robotPose = photonEstimator.estimateCoprocMultiTagPose(result)
                                .or(() -> photonEstimator.estimateLowestAmbiguityPose(result));

                if (robotPose.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = robotPose.get();
                Pose3d myRobotPose3d = estimatedRobotPose.estimatedPose.times(39.3700787402);
                Pose2d myRobotPose2d = myRobotPose3d.toPose2d();
                // System.out.println(myRobotPose3d);
                // System.out.println(myRobotPose2d);
                double x = myRobotPose2d.getX();
                double y = myRobotPose2d.getY();
                    if (Robot.alliance == "blue") {
                        distanceToHub = Math.sqrt((x - Constants.HubCoords.blueHubX)*(x - Constants.HubCoords.blueHubX)+(y - Constants.HubCoords.blueHubY)*(y - Constants.HubCoords.blueHubY));
                        SmartDashboard.putNumber("Distance to Hub: ", distanceToHub);
                        isblue = true;
                        SmartDashboard.putBoolean("Is Blue? ", isblue);
                    }
                    else if (Robot.alliance == "red") {
                        distanceToHub = Math.sqrt((x - Constants.HubCoords.redHubX)*(x - Constants.HubCoords.redHubX)+(y - Constants.HubCoords.redHubY)*(y - Constants.HubCoords.redHubY));
                        SmartDashboard.putNumber("Distance to Hub: ", distanceToHub);
                        isblue = false;
                        SmartDashboard.putBoolean("Is Blue? ", isblue);
                    }
                }
                // System.out.println(robotPose);
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    //System.out.println(target.getFiducialId());
                }
            } 

        } 
    }

    public Optional<EstimatedRobotPose> getRobotPose() {
        return robotPose;
    }

    public double getDistanceToHub() {
        SmartDashboard.putNumber("getDistanceToHub", distanceToHub);
        return distanceToHub;
    }
}
     