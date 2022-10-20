package frc.robot.UA6391;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public final class StatusFrameHelper {
    public static void statusFrameOff(TalonFX mc) {
        mc.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    }

    public static void statusFrameOff(TalonSRX mc) {
        mc.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    }

    public static void statusFrameOffExcept1(TalonFX mc) {
        mc.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
        mc.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    }

    public static void statusFrameOffExcept1(TalonSRX mc) {
        mc.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
        mc.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255);
        mc.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    }
}


