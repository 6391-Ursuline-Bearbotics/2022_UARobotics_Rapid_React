package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.FieldConstants;
import frc.robot.Constants.CAMERA;

public class PhotonVision {
   // Creates a new PhotonCamera.
   public PhotonCamera m_limelight = new PhotonCamera("limelight");
   public PhotonCamera m_HD3000 = new PhotonCamera("lifecam");
   public SimVisionSystem ballvisionSys;
   public SimVisionSystem shootervisionSys;

   public PhotonVision() {
      m_limelight.setPipelineIndex(CAMERA.LIMELIGHTPIPELINE);
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

      double tgtXPos = Units.feetToMeters(54 / 2);
      double tgtYPos = Units.feetToMeters(27 / 2);
      var targetPose = new Pose2d(new Translation2d(tgtXPos, tgtYPos), Rotation2d.fromDegrees(-21.0)); // meters
      double balltargetWidth = Units.inchesToMeters(9.5);
      double shootertargetWidth = Units.inchesToMeters(36); // Actually 4ft wide but this is a straight replacement for a curved goal
      double shootertargetHeight = Units.inchesToMeters(2);
      
      var ball = new Pose2d(FieldConstants.cargoD.getX(), FieldConstants.cargoD.getY(), new Rotation2d(0));
      var ballTgt = RectangularSimVisionTarget(ball,
                                       0,
                                       balltargetWidth,
                                       balltargetWidth, // same as height
                                       balltargetWidth);

      var shooterTgt = RectangularSimVisionTarget(targetPose,
                                       FieldConstants.visionTargetHeightLower,
                                       shootertargetWidth,
                                       shootertargetHeight,
                                       shootertargetWidth); // width is same as depth since 4ft circle
      
      for (int i = 0; i < ballTgt.size(); i++) {
         ballvisionSys.addSimVisionTarget(ballTgt.get(i));
      }
      for (int i = 0; i < shooterTgt.size(); i++) {
         shootervisionSys.addSimVisionTarget(shooterTgt.get(i));
      }

      NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setValue("v2022.1.4");
   }

   public void fieldSetup(Field2d field) {
      var ball = field.getObject("ball");
      var hub = field.getObject("hub");

      ball.setPose(FieldConstants.cargoD.getX(), FieldConstants.cargoD.getY(), Rotation2d.fromDegrees(0));
      hub.setPose(FieldConstants.hubCenter.getX(), FieldConstants.hubCenter.getY(), Rotation2d.fromDegrees(-21));
   }

   public void lightsOn() {
      m_limelight.setLED(VisionLEDMode.kOn);
   }

   public void lightsOff() {
      m_limelight.setLED(VisionLEDMode.kOff);
   }

   public double getYaw() {
      var result = m_limelight.getLatestResult();
      if (result.hasTargets()) {
         return result.getBestTarget().getYaw();
      }
      return -999.0;
   }

   // Both of these are dangerous and need "hasTargets" needs to be checked before using
   public double distanceToBallTarget(PhotonPipelineResult result) {
      return PhotonUtils.calculateDistanceToTargetMeters(CAMERA.BALLCAMERAHEIGHT,
                  CAMERA.BALLTARGETHEIGHT,
                  CAMERA.BALLCAMERAANGLE,
                  Units.degreesToRadians(result.getBestTarget().getPitch()));
   }

   public double distanceToShooterTarget(PhotonPipelineResult result) {
      return PhotonUtils.calculateDistanceToTargetMeters(CAMERA.SHOOTERCAMERAHEIGHT,
                  FieldConstants.visionTargetHeightLower,
                  CAMERA.SHOOTERCAMERAANGLE,
                  Units.degreesToRadians(result.getBestTarget().getPitch()));
   }

   public List<SimVisionTarget> RectangularSimVisionTarget(
         Pose2d targetPos,
         double targetHeightAboveGroundMeters,
         double targetWidthMeters,
         double targetHeightMeters,
         double targetDepthMeters) {
      List<SimVisionTarget> targetList = new ArrayList<SimVisionTarget>();
      
      var targetPos1 = targetPos.transformBy(new Transform2d(
         new Translation2d(0, -targetDepthMeters/2).rotateBy(targetPos.getRotation()),
         Rotation2d.fromDegrees(180.0)));
   
      var targetPos2 = targetPos.transformBy(new Transform2d(
         new Translation2d(-targetWidthMeters/2, 0).rotateBy(targetPos.getRotation()),
         Rotation2d.fromDegrees(-90.0)));

      var targetPos3 = targetPos.transformBy(new Transform2d(
         new Translation2d(0, targetDepthMeters/2).rotateBy(targetPos.getRotation()),
         Rotation2d.fromDegrees(0.0)));

      var targetPos4 = targetPos.transformBy(new Transform2d(
         new Translation2d(targetWidthMeters/2, 0).rotateBy(targetPos.getRotation()),
         Rotation2d.fromDegrees(90.0)));

      targetList.add(new SimVisionTarget(targetPos1, // Intial face
            targetHeightAboveGroundMeters,
            targetWidthMeters,
            targetHeightMeters));
      targetList.add(new SimVisionTarget(targetPos2, // Left face
            targetHeightAboveGroundMeters,
            targetDepthMeters, // On the side width is the inital depth
            targetHeightMeters));
      targetList.add(new SimVisionTarget(targetPos3, // Back face
            targetHeightAboveGroundMeters,
            targetWidthMeters,
            targetHeightMeters));
      targetList.add(new SimVisionTarget(targetPos4, // Right face
            targetHeightAboveGroundMeters,
            targetDepthMeters, // On the side width is the inital depth
            targetHeightMeters));
      return targetList;
   }
}
