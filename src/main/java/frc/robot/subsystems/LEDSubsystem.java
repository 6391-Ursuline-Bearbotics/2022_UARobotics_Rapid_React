package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led = new AddressableLED(LED.PWMPORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED.BUFFERSIZE);
  private int m_rainbowFirstPixelHue;

  private PhotonCamera ballCamera;
  private boolean redBall = false;
  private boolean blueBall = false;
  private Timer redTimer = new Timer();
  private double redTime = 0;
  private Timer blueTimer = new Timer();
  private double blueTime = 0;

  public LEDSubsystem(PhotonCamera ballCamera) {
    this.ballCamera = ballCamera;
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    int index = ballCamera.getPipelineIndex();
    PhotonPipelineResult result = ballCamera.getLatestResult();
    if (index == 0) {
      redBall(result.hasTargets());
      ballCamera.setPipelineIndex(1);
      SmartDashboard.putNumber("Red Timer", redTimer.get() - redTime);
      redTime = redTimer.get();
    }
    else { // should only have pipelines 0 & 1
      blueBall(result.hasTargets());
      ballCamera.setPipelineIndex(0);
      SmartDashboard.putNumber("Blue Timer", blueTimer.get() - blueTime);
      blueTime = blueTimer.get();
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

  public void redBall(boolean exists) {
    redBall = exists;
    setBallLEDs();
  }

  public void blueBall(boolean exists) {
    blueBall = exists;
    setBallLEDs();
  }

  private void setBallLEDs() {
    if (redBall && blueBall) {
      setHalf();
    }
    else if (redBall || blueBall) {
      if (redBall) {
        setAll(255, 5, 5);
      }
      if (blueBall) {
        setAll(5, 5, 255);
      }
    }
    else {
      setAll(0, 0, 0);
    }
  }

  private void setAll(int red, int green, int blue) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, blue);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setHalf() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i < m_ledBuffer.getLength() / 4) {
        m_ledBuffer.setRGB(i, 5, 5, 255);
      }
      else {
        m_ledBuffer.setRGB(i, 255, 5, 5);
      }
    }
    m_led.setData(m_ledBuffer);
  }
}