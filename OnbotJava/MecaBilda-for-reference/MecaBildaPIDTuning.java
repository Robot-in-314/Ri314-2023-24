package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name="MecaBildaPIDTuning", group="MecaBilda")
public class MecaBildaPIDTuning extends LinearOpMode {

    float rotate_angle = 0;
    double reset_angle = 0;
    
    private DcMotorEx front_left_wheel = null;
    private DcMotorEx back_left_wheel = null;
    private DcMotorEx back_right_wheel = null;
    private DcMotorEx front_right_wheel = null;

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        
        front_left_wheel = hardwareMap.get(DcMotorEx.class, "lf");
        back_left_wheel = hardwareMap.get(DcMotorEx.class, "lr");
        back_right_wheel = hardwareMap.get(DcMotorEx.class, "rr");
        front_right_wheel = hardwareMap.get(DcMotorEx.class, "rf");
        
        front_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        back_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        front_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        back_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE); // was REVERSE
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE); // was REVERSE
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD); // was FORWARD
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD); // was FORWARD
        
        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        waitForStart();

        double max_V = 0;  // going to be 2760
        
        /*
         * F = 32767 / 2760 = 11.8721
         * P = 0.1 * F = 1.1872
         * I = 0.1 * P = 0.1187
         * D = 0
         *
         * For position PIDF: P = 5.0
         */
        
//        while (opModeIsActive() && !isStopRequested()) {
        while (opModeIsActive()) {
            front_left_wheel.setPower(1);
            front_right_wheel.setPower(1);
            back_left_wheel.setPower(1);
            back_right_wheel.setPower(1);
            double v_fl = front_left_wheel.getVelocity();
            double v_fr = front_right_wheel.getVelocity();
            double v_bl = back_left_wheel.getVelocity();
            double v_br = back_right_wheel.getVelocity();
            double min_motor_V = Math.min(Math.min(v_fl, v_fr), Math.min(v_bl, v_br));
            if (min_motor_V > max_V) {max_V = min_motor_V;}
            telemetry.addData("max V", max_V);
            telemetry.update();
        }
    }
    
}
    
