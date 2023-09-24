package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecaBildaPIDUtil {

    public static double MaxVInTicks = 2760;
    
    public static void SetPIDCoefficients (DcMotorEx motor) {
        /* calculated in MecaBildaPIDTuning
         * see FIRST Global Motor PIDF Tuning Guide
         * https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.7kiv7jvkvdp8
         *
         * max V = 2760
         *
         * F = 32767 / 2760 = 11.8721
         * P = 0.1 * F = 1.1872
         * I = 0.1 * P = 0.1187
         * D = 0
         *
         * For position PIDF: P = 5.0
         */
        motor.setVelocityPIDFCoefficients(1.5, 0.1187, 0, 11.8721); // go up a little on P; it was sluggish
        motor.setPositionPIDFCoefficients(5.0);
    }
}