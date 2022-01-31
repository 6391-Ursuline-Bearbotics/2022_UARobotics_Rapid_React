package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CAMERA;

public class PhotonVision {
   // Creates a new PhotonCamera.
   public PhotonCamera m_limePhoton = new PhotonCamera("limelight");
   public PhotonCamera m_HD3000 = new PhotonCamera("lifecam");
   public SimVisionSystem ballvisionSys;
   public SimVisionSystem shootervisionSys;

   public PhotonVision() {
      m_limePhoton.setPipelineIndex(CAMERA.LIMELIGHTPIPELINE);
      m_HD3000.setPipelineIndex(CAMERA.HD3000PIPELINE);

      double camDiagFOV = 75.0; // degrees
      Transform2d cameraToRobot = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()); // meters
      double maxLEDRange = 20;          // meters
      int camResolutionWidth = 640;     // pixels
      int camResolutionHeight = 480;    // pixels
      double minTargetArea = 10;        // square pixels

      ballvisionSys = new SimVisionSystem("lifecam",
                                    camDiagFOV,
                                    CAMERA.BALLCAMERAANGLE,
                                    cameraToRobot,
                                    CAMERA.BALLCAMERAHEIGHT,
                                    maxLEDRange,
                                    camResolutionWidth,
                                    camResolutionHeight,
                                    minTargetArea);

      double tgtXPos = Units.feetToMeters(54);
      double tgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
      var targetPose = new Pose2d(new Translation2d(tgtXPos, tgtYPos), new Rotation2d(0.0)); // meters
      double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70);
      double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19);
      
      var newTgt = new SimVisionTarget(targetPose,
                                       CAMERA.BALLTARGEHEIGHT,
                                       targetWidth,
                                       targetHeight);
      
      ballvisionSys.addSimVisionTarget(newTgt);
   }

   public void lightsOn() {
      m_limePhoton.setLED(VisionLEDMode.kOn);
   }

   public void lightsOff() {
      m_limePhoton.setLED(VisionLEDMode.kOff);
   }

   public double getYaw() {
      var result = m_limePhoton.getLatestResult();
      if (result.hasTargets()) {
         return result.getBestTarget().getYaw();
      }
      return -999.0;
   }
}
