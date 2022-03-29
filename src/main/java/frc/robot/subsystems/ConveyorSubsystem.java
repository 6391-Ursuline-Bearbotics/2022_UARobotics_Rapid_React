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
    private final WPI_TalonSRX m_ConveyorMotor = new WPI_TalonSRX(CONVEYOR.CANID);

    DigitalInput frontconveyor = new DigitalInput(CONVEYOR.FRONTSENSORPORT);
    DigitalInput topconveyor = new DigitalInput(CONVEYOR.TOPSENSORPORT);

    XboxController6391 drv;
    XboxController6391 op;

    private boolean autoConvey = true;

    public ConveyorSubsystem(XboxController6391 drv, XboxController6391 op) {
        this.drv = drv;
        this.op = op;

        m_ConveyorMotor.setNeutralMode(NeutralMode.Brake);

        StatusFrameHelper.statusFrameOff(m_ConveyorMotor);
    }

    @Config
    public void turnOff() {
        this.m_ConveyorMotor.set(0);
        setAutoConvey(true);
    }

    @Config
    public void on(double speed) {
        setAutoConvey(false);
        this.m_ConveyorMotor.set(speed);
    }

    @Config
    public void turnBackwards() {
        setAutoConvey(false);
        this.m_ConveyorMotor.set(CONVEYOR.BACKSPEED);
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

    @Override
    public void periodic() {
        // Turn on the conveyor when the bottom sensor is blocked (ball waiting to go up)
        // unless top sensor blocked (the ball has no place to go)
        if (autoConvey) {
            if (!getTopConveyor() && getFrontConveyor()) {
                on(CONVEYOR.SPEED);
                drv.setRumble(0.8);
                op.setRumble(0.8);
            }          
            else {
                turnOff();
                drv.setRumble(0);
                op.setRumble(0);
            }
        }
    }
}