package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;
import frc.swervelib.SwerveDrivetrainModel;

public class LEDSubsystem extends SubsystemBase implements Loggable{
  private final AddressableLED m_led = new AddressableLED(LED.PWMPORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED.BUFFERSIZE);
  private int m_rainbowFirstPixelHue;

  private SwerveDrivetrainModel dt;
  private PhotonVision pv;
  private PhotonCamera ballCamera;
  private boolean shooter = false;
  private boolean redBall = false;
  private boolean blueBall = false;

  public LEDSubsystem(PhotonVision pv, SwerveDrivetrainModel dt) {
    this.dt = dt;
    this.pv = pv;
    this.ballCamera = pv.m_HD3000;
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      Alliance ally = DriverStation.getAlliance();
      if (ally.equals(Alliance.Red)) {
        ballCamera.setPipelineIndex(0);
        PhotonPipelineResult result = ballCamera.getLatestResult();
        redBall = result.hasTargets();
      }
      else if (ally.equals(Alliance.Blue)) { // should only have pipelines 0 & 1
        ballCamera.setPipelineIndex(1);
        PhotonPipelineResult result = ballCamera.getLatestResult();
        blueBall = result.hasTargets();
      }

      shooter = pv.m_limelight.getLatestResult().hasTargets();
      setLEDs();
    }
  }

  @Override
  public void simulationPeriodic() {
    pv.shootervisionSys.processFrame(dt.getCurActPose());
    pv.ballvisionSys.processFrame(dt.getCurActPose());
  }

  @Log
  public boolean anyTarget() {
    return ballCamera.getLatestResult().hasTargets();
  }

  @Log
  public double distanceToBall() {
    PhotonPipelineResult result = ballCamera.getLatestResult();
    if (result.hasTargets()) {
      return pv.distanceToBallTarget(result);
    }
    else {
      return 0.0;
    }
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

  private void setLEDs() {
    setBallLEDs();
    setShooterLEDs();
    m_led.setData(m_ledBuffer);
  }

  private void setBallLEDs() {
    if (redBall && blueBall) {
      setFrontHalf();
    }
    else if (redBall || blueBall) {
      if (redBall) {
        setFrontAll(Color.kRed);
      }
      if (blueBall) {
        setFrontAll(Color.kBlue);
      }
    }
    else {
      setFrontAll(Color.kBlack); // Off
    }
  }

  private void setShooterLEDs() {
    if (shooter) {
      setBackAll(Color.kGreen);
    }
    else {
      setBackAll(Color.kBlack); // Off
    }
  }

  private void setFrontAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  public void setFrontHalf() {
    for (int i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      if (i < m_ledBuffer.getLength() / 2) {
        m_ledBuffer.setLED(i, Color.kBlue);
      }
      else {
        m_ledBuffer.setLED(i, Color.kRed);
      }
    }
  }

  public void setBackAll(Color color) {
    for (var i = m_ledBuffer.getLength() / 2; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
  }
}