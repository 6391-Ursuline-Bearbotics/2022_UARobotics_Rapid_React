package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.swervelib.SwerveInput;

public class controlscheme {
    private final XboxController m_controller;
    private static final SendableChooser<String> driverChooser = new SendableChooser<>();

    private double m_translationX;
    private double m_translationY;
    private double m_rotation;

    public controlscheme(XboxController controller) {
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
              m_translationX = modifyAxis(-m_controller.getLeftY());
              m_translationY = modifyAxis(-m_controller.getLeftX());
              m_rotation = modifyAxis(-m_controller.getRightX());
              break;
            case "Left Stick and Triggers":
              m_translationX = modifyAxis(-m_controller.getLeftY());
              m_translationY = modifyAxis(-m_controller.getLeftX());
              m_rotation = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
              break;
            case "Split Sticks and Triggers":
              m_translationX = modifyAxis(-m_controller.getLeftY());
              m_translationY = modifyAxis(-m_controller.getRightX());
              m_rotation = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
              break;
            case "Gas Pedal":
              m_translationX = modifyAxis(-m_controller.getLeftY());
              m_translationY = modifyAxis(-m_controller.getLeftX());
              double angle = calculateTranslationDirection(m_translationX, m_translationY);
              m_translationX = Math.cos(angle) * m_controller.getRightTriggerAxis();
              m_translationY = Math.sin(angle) * m_controller.getRightTriggerAxis();
              m_rotation = modifyAxis(m_controller.getRightX());
              break;
        }
        return new SwerveInput(m_translationX, m_translationY, m_rotation);
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
    return Math.atan2(x, -y);
  }
}