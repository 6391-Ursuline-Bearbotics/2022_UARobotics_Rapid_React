package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import frc.robot.Constants.ControlPanelConstants;

public class ControlPanelSubsystem extends SubsystemBase implements Loggable{
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * Create the solenoid that will get the wheel and sensor into place
   */
  private final Solenoid drop = new Solenoid(PneumaticsModuleType.CTREPCM, ControlPanelConstants.kSolenoidPort);

  /**
   * Create a motor controller that will be used to spin the wheel
   */
  private final WPI_TalonSRX spinwheel = new WPI_TalonSRX(ControlPanelConstants.kSpinWheelPort);

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
   * The device will be automatically initialized with default parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();
  /**
   * Note: Any example colors should be calibrated as the user needs, these are
   * here as a basic example.
   */

  private ColorMatchResult matchedResult = new ColorMatchResult(Color.kBlack, 0);

  // Rev Color threshold
  // blue 0.143, 0.427, 0.429
  // green 0.197, 0.561, 0.240
  // red 0.561, 0.232, 0.114
  // yellow 0.361, 0.524, 0.113

  enum State {
    DISABLED,
    ENC_ROTATE,
    COLOR_ROTATE,
    COLOR_ROTATE_FINAL,
  };

  State state = State.DISABLED;
  double target_rotation = 0;

  public ControlPanelSubsystem() {
    m_colorMatcher.setConfidenceThreshold(0.80);
  }

  public void update() {
    if ( state == State.DISABLED ) {
        spinwheel.set(ControlMode.PercentOutput, 0);
    }
    else if ( state == State.ENC_ROTATE ) {
        if ( spinwheel.getSelectedSensorPosition() < ControlPanelConstants.kWheelSpeedFast ) {
            spinwheel.set(ControlMode.PercentOutput, ControlPanelConstants.kWheelSpeedFast);
        }
        else {
            state = State.DISABLED;
        }
    }
    else if ( state == State.COLOR_ROTATE_FINAL ) {
        if ( spinwheel.getSelectedSensorPosition() < target_rotation ) {
            spinwheel.set(ControlMode.PercentOutput, ControlPanelConstants.colorwheel_slow);
        }
        else
            state = State.DISABLED;
    }
  }

  public Color get_color() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and
     * can be useful if outputting the color to an RGB LED or similar. To read the
     * raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in well
     * lit conditions (the built in LED is a big help here!). The farther an object
     * is the more light from the surroundings will bleed into the measurements and
     * make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */
    //SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    //SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("Confidence", match.confidence);
    //SmartDashboard.putString("Detected Color", colorString);
    return match.color;
  }

  public void StartColorFind(double joystickval) {
    byte index;
    
    String gameData = DriverStation.getGameSpecificMessage();

    //get the color target
    if (gameData.length() > 0) {
        switch (gameData.charAt(0)) {
        case 'G':
            index = 0;
            break;
        case 'R':
            index = 1;
            break;
        case 'Y':
            index = 2;
            break;
        case 'B':
            index = 3;
            break;
        default:
            index = 100;
            break;
        }
    } else {
        index = 100;
    }

    //start the color finding
    if(index >= 4) {
        System.out.println("No color was provided to spin to.");
    } else {
        //need to rotate around the table.  We look at it from the far left, and need to rotate it 3 slots left, or 1 right, over to be what needs to be under the sensor.
        index = (byte)((index + 1) % 4);
    
        // no steps call real color find command
    }
  }

  @Log
  public void setOutput(double motorPercent) {
      this.spinwheel.set(motorPercent);
  }

  @Log
  public void rotateWheel() {
      // this will eventually either be a command or spin the wheeel 3 times.
  }
}