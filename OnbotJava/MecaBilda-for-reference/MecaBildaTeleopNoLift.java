// My fork: https://github.com/slylockfox/FGC_Guides/blob/master/driveJava.java

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="MecaBildaTeleopNoLift", group="MecaBilda")
public class MecaBildaTeleopNoLift extends LinearOpMode {

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
        
        front_left_wheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_left_wheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        front_right_wheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_right_wheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        front_left_wheel.setDirection(DcMotorEx.Direction.REVERSE); 
        back_left_wheel.setDirection(DcMotorEx.Direction.REVERSE); 
        front_right_wheel.setDirection(DcMotorEx.Direction.FORWARD); 
        back_right_wheel.setDirection(DcMotorEx.Direction.FORWARD); 
        
        front_left_wheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        MecaBildaPIDUtil.SetPIDCoefficients(front_left_wheel);
        MecaBildaPIDUtil.SetPIDCoefficients(back_left_wheel);
        MecaBildaPIDUtil.SetPIDCoefficients(front_right_wheel);
        MecaBildaPIDUtil.SetPIDCoefficients(back_right_wheel);
        
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

        
        while(opModeIsActive()){

                drive();
                resetAngle();
                //driveSimple();
                
                telemetry.update();
        }
    }
    
    public void driveSimple(){
        double power = 0.5;
        if(gamepad1.dpad_up){ //Forward
            front_left_wheel.setPower(-power);
            back_left_wheel.setPower(-power);
            back_right_wheel.setPower(-power);
            front_right_wheel.setPower(-power);
        }
        else if(gamepad1.dpad_left){ //Left
            front_left_wheel.setPower(power);
            back_left_wheel.setPower(-power);
            back_right_wheel.setPower(power);
            front_right_wheel.setPower(-power);
        }
        else if(gamepad1.dpad_down){ //Back
            front_left_wheel.setPower(power);
            back_left_wheel.setPower(power);
            back_right_wheel.setPower(power);
            front_right_wheel.setPower(power);
        }
        else if(gamepad1.dpad_right){ //Right
            front_left_wheel.setPower(-power);
            back_left_wheel.setPower(power);
            back_right_wheel.setPower(-power);
            front_right_wheel.setPower(power);
        }
        else if(Math.abs(gamepad1.right_stick_x) > 0){ //Rotation
            front_left_wheel.setPower(-gamepad1.right_stick_x);
            back_left_wheel.setPower(-gamepad1.right_stick_x);
            back_right_wheel.setPower(gamepad1.right_stick_x);
            front_right_wheel.setPower(gamepad1.right_stick_x);
        }
        else{
            front_left_wheel.setPower(0);
            back_left_wheel.setPower(0);
            back_right_wheel.setPower(0);
            front_right_wheel.setPower(0);
        }
    }

    public void drive() {
        double drivescale = 1.9; 
        double rotatescale = 1; 
        double Protate = -gamepad1.right_stick_x/rotatescale;
        double stick_x = -gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)*drivescale); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = -gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)*drivescale);
        double theta = 0;
        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
        if (gyroAngle <= 0) { 
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;
        
        if(gamepad1.right_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly. 
            gyroAngle = -Math.PI/2;
        }
        
        //Linear directions in case you want to do straight lines.
        if(gamepad1.dpad_right){
            stick_x = 0.7;
        }
        else if(gamepad1.dpad_left){
            stick_x = -0.7;
        }
        if(gamepad1.dpad_up){
            stick_y = -0.7;
        }
        else if(gamepad1.dpad_down){
            stick_y = 0.7;
        }
        
        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));
        
        telemetry.addData("Gyro", getHeading());
        
        double front_left_power = Py - Protate;
        double back_left_power = Px - Protate;
        double back_right_power = Py + Protate;
        double front_right_power = Px + Protate;
        front_left_wheel.setVelocity(front_left_power * MecaBildaPIDUtil.MaxVInTicks);
        back_left_wheel.setVelocity(back_left_power * MecaBildaPIDUtil.MaxVInTicks);
        back_right_wheel.setVelocity(back_right_power * MecaBildaPIDUtil.MaxVInTicks);
        front_right_wheel.setVelocity(front_right_power * MecaBildaPIDUtil.MaxVInTicks);

        // report velicity as a fraction
        telemetry.addData("lf V", front_left_wheel.getVelocity() / MecaBildaPIDUtil.MaxVInTicks );
        telemetry.addData("rf V", front_right_wheel.getVelocity() / MecaBildaPIDUtil.MaxVInTicks);
        telemetry.addData("lr V", back_left_wheel.getVelocity() / MecaBildaPIDUtil.MaxVInTicks);
        telemetry.addData("rr V", back_right_wheel.getVelocity() / MecaBildaPIDUtil.MaxVInTicks);
        
    }
    public void resetAngle(){
        if(gamepad1.a){
            reset_angle = getHeading() + reset_angle;
        }
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
    
