// Borrowed from 6328 here: https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/FieldConstants.java

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Contains various field dimensions and useful side points. All dimensions are in meters. */
public final class FieldConstants {

  // Field dimensions
  public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
  public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
  public static final double hangarLength = Units.inchesToMeters(128.75);
  public static final double hangarWidth = Units.inchesToMeters(116.0);

  // Vision target
  public static final double visionTargetDiameter =
      Units.inchesToMeters(4.0 * 12.0 + 5.375);
  public static final double visionTargetHeightLower =
      Units.inchesToMeters(8.0 * 12 + 5.625); // Bottom of tape
  public static final double visionTargetHeightUpper =
      visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape

  // Dimensions of hub and tarmac
  public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
  public static final Translation2d hubCenter =
      new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
  public static final double tarmacDiameter = Units.inchesToMeters(219.25); // Inner diameter
  public static final double tarmacFullSideLength =
      tarmacDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
  public static final double tarmacMarkedSideLength =
      Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
  public static final double tarmacMissingSideLength =
      tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff

  // Reference rotations (angle from hub to each reference point)
  public static final Rotation2d referenceARotation =
      Rotation2d.fromDegrees(180.0).minus(centerLineAngle)
          .plus(Rotation2d.fromDegrees(360.0 / 16.0));
  public static final Rotation2d referenceBRotation =
      referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d referenceCRotation =
      referenceBRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d referenceDRotation =
      referenceCRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));

  // Reference points (centered of the sides of the tarmac if they formed a complete octagon)
  public static final Pose2d referenceA =
      new Pose2d(hubCenter, referenceARotation).transformBy(
          transformFromTranslation(tarmacDiameter / 2.0, 0.0));
  public static final Pose2d referenceB =
      new Pose2d(hubCenter, referenceBRotation).transformBy(
          transformFromTranslation(tarmacDiameter / 2.0, 0.0));
  public static final Pose2d referenceC =
      new Pose2d(hubCenter, referenceCRotation).transformBy(
          transformFromTranslation(tarmacDiameter / 2.0, 0.0));
  public static final Pose2d referenceD =
      new Pose2d(hubCenter, referenceDRotation).transformBy(
          transformFromTranslation(tarmacDiameter / 2.0, 0.0));

  // Cargo points
  public static final double cornerToCargoY = Units.inchesToMeters(15.56);
  public static final double referenceToCargoY =
      (tarmacFullSideLength / 2.0) - cornerToCargoY;
  public static final double referenceToCargoX = Units.inchesToMeters(40.44);
  public static final Pose2d cargoA = referenceA.transformBy(
      transformFromTranslation(referenceToCargoX, -referenceToCargoY));
  public static final Pose2d cargoB = referenceA.transformBy(
      transformFromTranslation(referenceToCargoX, referenceToCargoY));
  public static final Pose2d cargoC = referenceB.transformBy(
      transformFromTranslation(referenceToCargoX, referenceToCargoY));
  public static final Pose2d cargoD = referenceC.transformBy(
      transformFromTranslation(referenceToCargoX, -referenceToCargoY));
  public static final Pose2d cargoE = referenceD.transformBy(
      transformFromTranslation(referenceToCargoX, -referenceToCargoY));
  public static final Pose2d cargoF = referenceD.transformBy(
      transformFromTranslation(referenceToCargoX, referenceToCargoY));

    /**
     * Creates a pure translating transform
     * 
     * @param x The x componenet of the translation
     * @param y The y componenet of the translation
     * @return The resulting transform
     */
    public static Transform2d transformFromTranslation(double x, double y) {
        return new Transform2d(new Translation2d(x, y), new Rotation2d());
    }
}
