package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LauncherSS;
import frc.robot.subsystems.VisionSS;
public class LauncherC extends Command {
    //public double speed;
    // List<PhotonPipelineResult> results = RobotContainer.rc_visionSS.camera.getAllUnreadResults();
    // PhotonTrackedTarget target = results.get(0).getBestTarget();
    // double targetHypotenuse = 1;

    // public final double getDistance() {
    //     if (results.size() > 0) {
    //         final double targetPitchRadians = target.getPitch();
    //         targetHypotenuse = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.cameraHeightMeters, Constants.VisionConstants.targetHeightMeters, Constants.VisionConstants.cameraPitchRadians, targetPitchRadians);
    //     } 
    //     return targetHypotenuse;
    // }
   


    public LauncherC(LauncherSS subsystem) {
        subsystem = RobotContainer.rc_launcherSS;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //double launchspeed = RobotContainer.rc_launcherSS.calculateLaunchSpeed(getDistance());
        //RobotContainer.rc_launcherSS.spin(launchspeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.rc_launcherSS.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
