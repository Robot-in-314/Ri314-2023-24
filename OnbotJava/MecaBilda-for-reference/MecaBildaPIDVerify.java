package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.Set;
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
@Autonomous(name="MecaBildaPIDVerify", group="MecaBilda")
public class MecaBildaPIDVerify extends LinearOpMode {

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
        
        front_left_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        back_left_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        front_right_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        back_right_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE); 
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        
        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        MecaBildaPIDUtil.SetPIDCoefficients(front_left_wheel);
        MecaBildaPIDUtil.SetPIDCoefficients(back_left_wheel);
        MecaBildaPIDUtil.SetPIDCoefficients(front_right_wheel);
        MecaBildaPIDUtil.SetPIDCoefficients(back_right_wheel);
        
        waitForStart();
        
        if (opModeIsActive()){
            double half_speed_in_ticks = 1380; // see MecaBildaPIDTuning
            front_left_wheel.setVelocity(half_speed_in_ticks);
            front_right_wheel.setVelocity(half_speed_in_ticks);
            back_left_wheel.setVelocity(half_speed_in_ticks);
            back_right_wheel.setVelocity(half_speed_in_ticks);            
        }

        while (opModeIsActive()) {
            telemetry.addData("FL", front_left_wheel.getVelocity());
            telemetry.addData("FR", front_right_wheel.getVelocity());
            telemetry.addData("BL", back_left_wheel.getVelocity());
            telemetry.addData("BR", back_right_wheel.getVelocity());
            telemetry.update();
        }
    }
    
}
    
