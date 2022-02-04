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
import frc.robot.FieldConstants;
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

      double ballcamDiagFOV = 75.0; // degrees
      double shootercamDiagFOV = 75.0; // degrees
      Transform2d ballcameraToRobot = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()); // meters
      Transform2d shootercameraToRobot = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()); // meters
      double maxLEDRange = 20;          // meters
      int ballcamResolutionWidth = 640;     // pixels
      int ballcamResolutionHeight = 480;    // pixels
      double ballminTargetArea = 10;        // square pixels
      int shootercamResolutionWidth = 640;     // pixels
      int shootercamResolutionHeight = 480;    // pixels
      double shooterminTargetArea = 10;        // square pixels
      

      ballvisionSys = new SimVisionSystem("lifecam",
                                    ballcamDiagFOV,
                                    CAMERA.BALLCAMERAANGLE,
                                    ballcameraToRobot,
                                    CAMERA.BALLCAMERAHEIGHT,
                                    9000, // does not use LEDs
                                    ballcamResolutionWidth,
                                    ballcamResolutionHeight,
                                    ballminTargetArea);

      shootervisionSys = new SimVisionSystem("limelight",
                                    shootercamDiagFOV,
                                    CAMERA.SHOOTERCAMERAANGLE,
                                    shootercameraToRobot,
                                    CAMERA.SHOOTERCAMERAHEIGHT,
                                    maxLEDRange,
                                    shootercamResolutionWidth,
                                    shootercamResolutionHeight,
                                    shooterminTargetArea);

      double tgtXPos = Units.feetToMeters(54);
      double tgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
      var targetPose = new Pose2d(new Translation2d(tgtXPos, tgtYPos), new Rotation2d(0.0)); // meters
      double balltargetWidth = Units.inchesToMeters(9.5);
      double shootertargetWidth = Units.inchesToMeters(36); // Actually 4ft wide but this is a straight replacement for a curved goal
      double shootertargetHeight = Units.inchesToMeters(2);
      
      var ballTgt = new SimVisionTarget(FieldConstants.cargoD,
                                       CAMERA.BALLTARGETHEIGHT,
                                       balltargetWidth,
                                       balltargetWidth); // same as height

      var shooterTgt = new SimVisionTarget(targetPose,
                                       FieldConstants.visionTargetHeightLower,
                                       shootertargetWidth,
                                       shootertargetHeight);
      
      ballvisionSys.addSimVisionTarget(ballTgt);
      shootervisionSys.addSimVisionTarget(shooterTgt);
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
