package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="MecaBildaPIDLoggingAuto", group="MecaBilda")
public class MecaBildaPIDLoggingAuto extends LinearOpMode {

    float rotate_angle = 0;
    double reset_angle = 0;
    
    private DcMotorExLogged front_left_wheel = null;
    private DcMotorExLogged back_left_wheel = null;
    private DcMotorExLogged back_right_wheel = null;
    private DcMotorExLogged front_right_wheel = null;

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        
        front_left_wheel = new DcMotorExLogged (hardwareMap.get(DcMotorEx.class, "lf"), "velocitiesFL");
        back_left_wheel = new DcMotorExLogged (hardwareMap.get(DcMotorEx.class, "lr"),  "velocitiesBL");
        back_right_wheel = new DcMotorExLogged (hardwareMap.get(DcMotorEx.class, "rr"), "velocitiesBR");
        front_right_wheel = new DcMotorExLogged (hardwareMap.get(DcMotorEx.class, "rf"),"velocitiesFR");
        
        front_left_wheel.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_left_wheel.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_right_wheel.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_right_wheel.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        front_left_wheel.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_left_wheel.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right_wheel.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right_wheel.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        front_left_wheel.motor.setDirection(DcMotorEx.Direction.REVERSE); 
        back_left_wheel.motor.setDirection(DcMotorEx.Direction.REVERSE); 
        front_right_wheel.motor.setDirection(DcMotorEx.Direction.FORWARD); 
        back_right_wheel.motor.setDirection(DcMotorEx.Direction.FORWARD); 
        
        front_left_wheel.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_left_wheel.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_right_wheel.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_right_wheel.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        MecaBildaPIDUtil.SetPIDCoefficients(front_left_wheel.motor);
        MecaBildaPIDUtil.SetPIDCoefficients(back_left_wheel.motor);
        MecaBildaPIDUtil.SetPIDCoefficients(front_right_wheel.motor);
        MecaBildaPIDUtil.SetPIDCoefficients(back_right_wheel.motor);
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // remap axes in order to  use with vertically mounted REV hub
        
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100); //Changing modes requires a delay before doing anything else
        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100); //Changing modes again requires a delay
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        while(!opModeIsActive() && !isStopRequested()){} // like waitforstart

        if(opModeIsActive()){
                driveSequence();
                telemetry.update();
        }
    }
    
    public void driveSequence(){
        double half_speed_in_ticks = 1380; // see MecaBildaPIDTuning
        double full_speed_in_ticks = half_speed_in_ticks * 2;
        
        // Drive forward, at first slowly, then faster
        while (opModeIsActive() && front_left_wheel.motor.getCurrentPosition() < 1000) {
            front_left_wheel.setVelocity(half_speed_in_ticks);
            front_right_wheel.setVelocity(half_speed_in_ticks);
            back_left_wheel.setVelocity(half_speed_in_ticks);
            back_right_wheel.setVelocity(half_speed_in_ticks);  
        }
        while (opModeIsActive() && front_left_wheel.motor.getCurrentPosition() < 2000) {
            front_left_wheel.setVelocity(full_speed_in_ticks);  
            front_right_wheel.setVelocity(full_speed_in_ticks); 
            back_left_wheel.setVelocity(full_speed_in_ticks);   
            back_right_wheel.setVelocity(full_speed_in_ticks);  
        }
        front_left_wheel.setVelocity(0);
        back_left_wheel.setVelocity(0);
        back_right_wheel.setVelocity(0);
        front_right_wheel.setVelocity(0);
        
        sleep(1000); // wait 1 sec
        
        // Rotate 90 degrees left
        double gyroAngle = getHeading();
        while (opModeIsActive() && !(gyroAngle > 85 && gyroAngle < 95)) { // not between 85 and 95
            telemetry.addData("Gyro", gyroAngle);
            telemetry.update();
            front_left_wheel.setVelocity(-half_speed_in_ticks);
            front_right_wheel.setVelocity(half_speed_in_ticks);
            back_left_wheel.setVelocity(-half_speed_in_ticks);
            back_right_wheel.setVelocity(half_speed_in_ticks);  
            gyroAngle = getHeading();
        }
        front_left_wheel.setVelocity(0);
        back_left_wheel.setVelocity(0);
        back_right_wheel.setVelocity(0);
        front_right_wheel.setVelocity(0);
        
        sleep(1000); // wait 1 sec
        
        // Strafe left, at first slowly, then faster
        front_left_wheel.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        while (front_left_wheel.motor.getCurrentPosition() > -1000) {
            front_left_wheel.setVelocity(-half_speed_in_ticks);
            front_right_wheel.setVelocity(half_speed_in_ticks);
            back_left_wheel.setVelocity(half_speed_in_ticks);
            back_right_wheel.setVelocity(-half_speed_in_ticks); 
        }
        while (front_left_wheel.motor.getCurrentPosition() > -2000) {
            front_left_wheel.setVelocity(-full_speed_in_ticks);
            front_right_wheel.setVelocity(full_speed_in_ticks);
            back_left_wheel.setVelocity(full_speed_in_ticks);
            back_right_wheel.setVelocity(-full_speed_in_ticks); 
        }
        front_left_wheel.setVelocity(0);
        back_left_wheel.setVelocity(0);
        back_right_wheel.setVelocity(0);
        front_right_wheel.setVelocity(0);
                
        sleep(1000); // wait 1 sec
        
        // Rotate back to zero
        gyroAngle = getHeading();
        while (opModeIsActive() && !(gyroAngle > -5 && gyroAngle < 5)) { // not between 85 and 95
            telemetry.addData("Gyro", gyroAngle);
            telemetry.update();
            front_left_wheel.setVelocity(half_speed_in_ticks);
            front_right_wheel.setVelocity(-half_speed_in_ticks);
            back_left_wheel.setVelocity(half_speed_in_ticks);
            back_right_wheel.setVelocity(-half_speed_in_ticks);  
            gyroAngle = getHeading();
        }
        
        // done with sequence
    }

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }
}
    
