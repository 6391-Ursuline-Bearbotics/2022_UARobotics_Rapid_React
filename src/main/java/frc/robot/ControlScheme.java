package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.UA6391.XboxController6391;
import frc.swervelib.SwerveInput;

public class ControlScheme {
    private final XboxController6391 m_controller;
    private static final SendableChooser<String> driverChooser = new SendableChooser<>();

    private SlewRateLimiter slewX = new SlewRateLimiter(10);
    private SlewRateLimiter slewY = new SlewRateLimiter(10);
    private SlewRateLimiter slewRot = new SlewRateLimiter(10);

    private double m_translationX;
    private double m_translationY;
    private double m_rotation;

    public ControlScheme(XboxController6391 controller) {
        this.m_controller = controller;

        // Control Scheme Chooser
        driverChooser.setDefaultOption("Both Sticks", "Both Sticks");
        driverChooser.addOption("Left Stick and Triggers", "Left Stick and Triggers");
        driverChooser.addOption("Split Sticks and Triggers", "Split Sticks and Triggers");
        driverChooser.addOption("Gas Pedal", "Gas Pedal");
        SmartDashboard.putData("Driver Chooser", driverChooser);
    }

    public SwerveInput getJoystickSpeeds() {
        switch (driverChooser.getSelected()) {
            case "Both Sticks":
              m_translationX = slewX.calculate(modifyAxis(-m_controller.JoystickLY()));
              m_translationY = slewY.calculate(modifyAxis(-m_controller.JoystickLX()));
              m_rotation = slewRot.calculate(modifyAxis(-m_controller.JoystickRX()));
              break;
            case "Left Stick and Triggers":
              m_translationX = slewX.calculate(modifyAxis(-m_controller.JoystickLY()));
              m_translationY = slewY.calculate(modifyAxis(-m_controller.JoystickLX()));
              m_rotation = slewRot.calculate(m_controller.TriggerCombined());
              break;
            case "Split Sticks and Triggers":
              m_translationX = slewX.calculate(modifyAxis(-m_controller.JoystickLY()));
              m_translationY = slewY.calculate(modifyAxis(-m_controller.JoystickRX()));
              m_rotation = slewRot.calculate(m_controller.TriggerCombined());
              break;
            case "Gas Pedal":
              m_translationX = modifyAxis(-m_controller.JoystickLY());
              m_translationY = modifyAxis(-m_controller.JoystickLX());
              double angle = calculateTranslationDirection(m_translationX, m_translationY);
              m_translationX = slewX.calculate(Math.cos(angle) * m_controller.TriggerR());
              m_translationY = slewY.calculate(Math.sin(angle) * m_controller.TriggerR());
              m_rotation = slewRot.calculate(modifyAxis(-m_controller.JoystickRX()));
              break;
        }
        return new SwerveInput(m_translationX, m_translationY, m_rotation);
    }

    public void modifySlew(double slewX, double slewY, double slewRotate) {
        this.slewX = new SlewRateLimiter(slewX);
        this.slewY = new SlewRateLimiter(slewY);
        this.slewRot = new SlewRateLimiter(slewRotate);
    }

  private static double modifyAxis(double value) {
      // Square the axis
      value = Math.copySign(value * value, value);

      return value;
  }

  /**
     * Calculates the angle of translation set by the left stick.
     *
     * @return The angle of translation. 0 corresponds to forwards, and positive
     *     corresponds to counterclockwise.
    */
  private double calculateTranslationDirection(double x, double y) {
    // Calculate the angle.
    // Swapping x/y and inverting y because our coordinate system has +x forwards and -y right
    return Math.atan2(y, x);
  }
}