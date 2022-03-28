// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlScheme;
import frc.robot.FieldConstants;
import frc.robot.Constants.CAMERA;
import frc.robot.Constants.DRIVE;
import frc.robot.subsystems.PhotonVision;
import frc.swervelib.SwerveSubsystem;

public class AutoAim extends CommandBase {
  PhotonVision m_PhotonVision;
  SwerveSubsystem m_swerve;
  Boolean m_shooter;
  PhotonPipelineResult result;
  ControlScheme m_scheme;
  PIDController m_rotationPID = new PIDController(DRIVE.AIMkP, 0, 0);

  public AutoAim(SwerveSubsystem swerve, PhotonVision PhotonVision, Boolean shooter, ControlScheme scheme) {
    m_swerve = swerve;
    m_PhotonVision = PhotonVision;
    m_shooter = shooter;
    m_scheme = scheme;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_shooter) {
      m_PhotonVision.lightsOn();
    }
    else {
      m_PhotonVision.lightsOn();
    }

    m_rotationPID.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the latest result from the correct camera
    if (m_shooter) {
      result = m_PhotonVision.m_limelight.getLatestResult();
    }
    else {
      result = m_PhotonVision.m_HD3000.getLatestResult();
    }

    // Make sure it has a target we can use
    SmartDashboard.putBoolean("hastargets", result.hasTargets());
    if (result.hasTargets()) {
      // Get the angle of the best target needs to be inverted because Photon is + right
      var targetAngle = result.getBestTarget().getYaw();
      SmartDashboard.putNumber("targetangle", targetAngle);

      // Get the distance to the Target
      double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA.SHOOTERCAMERAHEIGHT, FieldConstants.visionTargetHeightLower,
          Units.degreesToRadians(CAMERA.SHOOTERCAMERAANGLE), Units.degreesToRadians(result.getBestTarget().getPitch()));
      SmartDashboard.putNumber("distanceToTarget", Units.metersToFeet(range));

      // Get the drive inputs we are using
      var input = m_scheme.getJoystickSpeeds();
      // Override the rotation input with a PID value seeking centered in the camera
      var pidAngle = m_rotationPID.calculate(targetAngle);
      input.m_rotation = Math.copySign(DRIVE.AIMFF + Math.abs(pidAngle), pidAngle);
      
      // If aiming at a ball we want to use robot relative movement
      if (m_shooter) {
        m_swerve.dt.setModuleStates(input);
      }
      else {
        m_swerve.dt.setModuleStates(DRIVE.KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds(
                                input.m_translationX * DRIVE.MAX_FWD_REV_SPEED_MPS_EST,
                                input.m_translationY * DRIVE.MAX_FWD_REV_SPEED_MPS_EST,
                                input.m_rotation * DRIVE.MAX_ROTATE_SPEED_RAD_PER_SEC_EST
                        )  
                )
        );
      }
    }
    else {
      // Just use normal robot controls
      m_swerve.dt.setModuleStates(m_scheme.getJoystickSpeeds());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_PhotonVision.lightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
