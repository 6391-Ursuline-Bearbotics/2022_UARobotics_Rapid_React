package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.CONVEYOR;
import frc.robot.UA6391.StatusFrameHelper;
import frc.robot.UA6391.XboxController6391;

public class ConveyorSubsystem extends SubsystemBase implements Loggable{
    public final WPI_TalonSRX m_ConveyorMotor = new WPI_TalonSRX(CONVEYOR.CANID);

    DigitalInput frontconveyor = new DigitalInput(CONVEYOR.FRONTSENSORPORT);
    DigitalInput topconveyor = new DigitalInput(CONVEYOR.TOPSENSORPORT);

    XboxController6391 drv;
    XboxController6391 op;

    @Log
    private boolean autoConvey = true;

    public ConveyorSubsystem(XboxController6391 drv, XboxController6391 op) {
        this.drv = drv;
        this.op = op;

        m_ConveyorMotor.setNeutralMode(NeutralMode.Brake);

        StatusFrameHelper.statusFrameOff(m_ConveyorMotor);
    }

    @Config
    public void turnOff() {
        m_ConveyorMotor.set(0);
        setAutoConvey(true);
    }

    @Config
    public void on(double speed) {
        setAutoConvey(false);
        m_ConveyorMotor.set(speed);
    }

    @Config
    public void turnBackwards() {
        setAutoConvey(false);
        m_ConveyorMotor.set(CONVEYOR.BACKSPEED);
    }

    @Log
    @Log(tabName = "Dashboard", name = "Front Sensor")
    public boolean getFrontConveyor() {
        return !frontconveyor.get();
    }

    @Log
    @Log(tabName = "Dashboard", name = "Top Sensor")
    public boolean getTopConveyor() {
        return !topconveyor.get();
    }

    @Config
    public void setAutoConvey(boolean value) {
        autoConvey = value;
    }

    public boolean getAutoConvey() {
        return autoConvey;
    }
}