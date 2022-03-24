package frc.robot.UA6391;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public final class StatusFrameHelper {
    public static void statusFrameOff(TalonFX mc) {
        mc.setStatusFramePeriod(StatusFrame.Status_1_General, 244);
        mc.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 245);
        mc.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 246);
        mc.setStatusFramePeriod(StatusFrame.Status_6_Misc, 247);
        mc.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 248);
        mc.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 249);
        mc.setStatusFramePeriod(StatusFrame.Status_10_Targets, 250);
        mc.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 251);
        mc.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 252);
        mc.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 253);
        mc.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 254);
        mc.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    }

    public static void statusFrameOff(TalonSRX mc) {
        mc.setStatusFramePeriod(StatusFrame.Status_1_General, 244);
        mc.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 245);
        mc.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 246);
        mc.setStatusFramePeriod(StatusFrame.Status_6_Misc, 247);
        mc.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 248);
        mc.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 249);
        mc.setStatusFramePeriod(StatusFrame.Status_10_Targets, 250);
        mc.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 251);
        mc.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 252);
        mc.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 253);
        mc.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 254);
        mc.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    }
}


