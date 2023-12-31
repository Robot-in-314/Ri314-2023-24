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
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

enum IntakeState {
  kIntakeUp1,
  kIntakeUp2,
  kIntakeDown1,
  kIntakeDown2,
  kClimberReleased1,
  kClimberReleased2
}

enum SlideState {
  kIntakeReady,
  kSlideDownClearingBucket,
  kSlideDownBucketUp,
  kSlideManualControl,
  kSlideAutomaticControl,
  kSlideExtendingAuto,
  kSlideRetractingAuto,
  kSlideStowingBucket1,
  kSlideStowingBucket2,
  kSlideStowingBucket3,
  kSlideStowingBucket4,
  kSlideStowingBucket5,
  kSlideStowingBucket6,
  kSlideStowingBucket7,
  kSlideStowingBucket8,
  kSlideExtended
};



@TeleOp(name="Ri314Teleop", group="Ri314")

public class Ri314Teleop extends LinearOpMode {

    private static final double kArmStowedPosition = 0.66;
    private static final double kArmTravelPosition = 0.4;

    private static final double kIntakeRelease = 0.35;
    private static final double kClimberRelease = 0;

    private static final int kSlideExtended = 2570; // 3300 is full extension, but for scoring...
    private static final int kSlideRetracted1 = 300;
    private static final int kSlideRetracted2 = 0;

    float rotate_angle = 0;
    double reset_angle = 0;
    
    private DcMotorEx front_left_wheel = null;
    private DcMotorEx back_left_wheel = null;
    private DcMotorEx back_right_wheel = null;
    private DcMotorEx front_right_wheel = null;

    private DcMotorEx climber_motor = null;
    private DcMotorEx slide_motor = null;

    private Servo launcher_servo = null;
    private Servo intake_servo = null;
    private Servo bucket_servo = null;
    private Servo arm_servo = null;
    private CRServo intake_motor = null;

    private IntakeState intake_dropper_state = IntakeState.kIntakeUp1;
    private SlideState slide_state = SlideState.kIntakeReady;
    private ElapsedTime bucket_timer = new ElapsedTime();
    private DigitalChannel line_sensor = null;

    boolean slide_homed = false;

    BNO055IMU imu;

    @Override
    public void runOpMode() {
        
        front_left_wheel = hardwareMap.get(DcMotorEx.class, "lf");
        back_left_wheel = hardwareMap.get(DcMotorEx.class, "lr");
        back_right_wheel = hardwareMap.get(DcMotorEx.class, "rr");
        front_right_wheel = hardwareMap.get(DcMotorEx.class, "rf");

        climber_motor = hardwareMap.get(DcMotorEx.class, "climber");
        slide_motor = hardwareMap.get(DcMotorEx.class, "elevator");
        intake_motor = hardwareMap.get(CRServo.class, "intakeMotor");
        
        launcher_servo = hardwareMap.get(Servo.class, "launcherServo");
        intake_servo = hardwareMap.get(Servo.class, "intakeDropper");
        arm_servo = hardwareMap.get(Servo.class, "arm");

        line_sensor = hardwareMap.get(DigitalChannel.class, "line2");
        line_sensor.setMode(DigitalChannel.Mode.INPUT);
        
        front_left_wheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        back_left_wheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        front_right_wheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        back_right_wheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);

        climber_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODERS);
        slide_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);

        front_left_wheel.setDirection(DcMotorEx.Direction.FORWARD); 
        back_left_wheel.setDirection(DcMotorEx.Direction.REVERSE); 
        front_right_wheel.setDirection(DcMotorEx.Direction.FORWARD); 
        back_right_wheel.setDirection(DcMotorEx.Direction.FORWARD); 
        
        front_left_wheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        climber_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Ri314PIDUtil.SetPIDCoefficients(front_left_wheel);
        Ri314PIDUtil.SetPIDCoefficients(back_left_wheel);
        Ri314PIDUtil.SetPIDCoefficients(front_right_wheel);
        Ri314PIDUtil.SetPIDCoefficients(back_right_wheel);
        
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

        check_slide_homed();

        while(!opModeIsActive() && !isStopRequested()){} // like waitforstart

        
        while(opModeIsActive()){

                drive();
                resetAngle();
                
                move_slide();
                deploy_intake();
                launch_airplane();
                climb();
                
                telemetry.addData("Slide Homed", slide_homed);
                telemetry.update();
        }
    }
    
    private boolean slide_at_bottom_limit() {
        return !line_sensor.getState();
    }

    private void deploy_intake() {
        switch (intake_dropper_state) {
            case kIntakeUp1:
                if (gamepad1.x) {
                    intake_dropper_state = IntakeState.kIntakeUp2;
                }
                break;
            case kIntakeUp2:
                if (!gamepad1.x) { // move servo on button release
                    intake_servo.setPosition(kIntakeRelease);
                    intake_dropper_state = IntakeState.kIntakeDown1;
                }
                break;
            case kIntakeDown1:
                if (gamepad1.x) {
                    intake_dropper_state = IntakeState.kIntakeDown2;
                }
                break;
            case kIntakeDown2:
                if (!gamepad1.x) { // move servo on button release
                    intake_servo.setPosition(kClimberRelease);
                    intake_dropper_state = IntakeState.kClimberReleased1;
                }
                break;
            case kClimberReleased1:
                if (gamepad1.x) {
                    intake_dropper_state = IntakeState.kClimberReleased2;
                }
                break;
            case kClimberReleased2:
                if (!gamepad1.x) { // move servo on button release
                    intake_servo.setPosition(kIntakeRelease);
                    intake_dropper_state = IntakeState.kIntakeDown1;
                }
                break;
        }
    }

    private boolean check_for_return_to_slide_ready () {
        boolean result = false;
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.y) { // abort back to square 1
            slide_state = SlideState.kIntakeReady;
            slide_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
            slide_motor.setVelocity(0);
            result = true;
        }
        return result;
    }

    private void bucket_stow_move_step (double target_pos, SlideState next_step) {
        if (!check_for_return_to_slide_ready()) {
            if (slide_homed && gamepad1.a) { 
                arm_servo.setPosition(target_pos);
                bucket_timer.reset();
                slide_state = next_step;
            }
        }
    }

    private void bucket_stow_wait_step (double wait_time, SlideState next_step) {
        if (!check_for_return_to_slide_ready()) {
            if (bucket_timer.seconds() < wait_time) { // wait in this state until bucket has time to stow 
                slide_motor.setPower(0);
            } else { // done waiting; move on
                slide_state = next_step;
            }
        }
    }

    private void move_slide() {
        telemetry.addData("Slide V: ", slide_motor.getVelocity());
        telemetry.addData("Slide Pos: ", slide_motor.getCurrentPosition());
        switch (slide_state) {
            case kIntakeReady:
                telemetry.addData("Slide State: ", "READY");
                if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.y) { // move slide manually or automatically; bucket must be up
                    intake_motor.setPower(0); // leaving ready state, so stop intake
                    arm_servo.setPosition(kArmTravelPosition);
                    bucket_timer.reset();
                    slide_state = SlideState.kSlideDownClearingBucket;
                } else if (gamepad1.b) { // run intake in reverse
                    if (intake_motor.getPower() == 0) {
                        intake_motor.setPower(1); 
                    } else {
                        intake_motor.setPower(0); 
                    }
                } else if (gamepad1.a) { // toggle intake motor
                    if (intake_motor.getPower() == 0) {
                        intake_motor.setPower(-1); 
                    } else {
                        intake_motor.setPower(0); 
                    }
                }
                break;
            case kSlideDownClearingBucket:
                telemetry.addData("Slide State: ", "CLEARING");
                if (bucket_timer.seconds() < 1) { // wait in this state until bucket has time to raise 
                    slide_motor.setPower(0);
                } else { // done waiting; move on
                    slide_state = SlideState.kSlideDownBucketUp;
                }
                break;
            case kSlideDownBucketUp:
                telemetry.addData("Slide State: ", "DOWNUP");
                if (gamepad1.dpad_up || gamepad1.dpad_down) {
                    slide_state = SlideState.kSlideManualControl;
                } else if (slide_homed && (gamepad1.a || gamepad1.y)) {
                    slide_state = SlideState.kSlideAutomaticControl;
                } else {
                    slide_motor.setPower(0);
                }
                break;
            case kSlideAutomaticControl:
                telemetry.addData("Slide State: ", "AUTO");
                if (gamepad1.dpad_up || gamepad1.dpad_down) {
                    slide_state = SlideState.kSlideManualControl;
                } else if (slide_homed) {
                    if (gamepad1.y) {
                        slide_motor.setTargetPosition(kSlideExtended);
                        slide_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        slide_motor.setVelocity(2000);
                        slide_state = SlideState.kSlideExtendingAuto;
                    } else if (gamepad1.a) {
                        slide_motor.setTargetPosition(kSlideRetracted1);
                        slide_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        slide_motor.setVelocity(1000);
                        slide_state = SlideState.kSlideRetractingAuto;
                    } else {
                        slide_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
                        slide_motor.setVelocity(0);
                    }
                }
                // allow right trigger to dump bucket
                if (!gamepad1.right_bumper) {
                    arm_servo.setPosition(0.4 - gamepad1.right_trigger/2);
                }
                break;
            case kSlideExtendingAuto:
                telemetry.addData("Slide State: ", "MOVING");
                if (gamepad1.dpad_up || gamepad1.dpad_down) {
                    slide_state = SlideState.kSlideManualControl;
                } else if (slide_homed) {
                    if (gamepad1.y) {
                        if (!slide_motor.isBusy()) { // completed movement
                            slide_state = SlideState.kSlideAutomaticControl;
                        }
                    } else {
                        slide_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
                        slide_motor.setVelocity(0);
                        slide_state = SlideState.kSlideAutomaticControl;
                    }
                }
                // allow right trigger to dump bucket
                if (!gamepad1.right_bumper) {
                    arm_servo.setPosition(0.4 - gamepad1.right_trigger/2);
                }
                break;
            case kSlideRetractingAuto:
                telemetry.addData("Slide State: ", "RETRACTING");
                if (gamepad1.dpad_up || gamepad1.dpad_down) {
                    slide_state = SlideState.kSlideManualControl;
                } else if (slide_homed) {
                    if (gamepad1.a) {
                        if (!slide_motor.isBusy()) { // completed movement
                            arm_servo.setPosition(0.8);
                            slide_state = SlideState.kSlideStowingBucket1;
                        }
                    } else {
                        slide_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
                        slide_motor.setVelocity(0);
                        slide_state = SlideState.kSlideAutomaticControl;
                    }
                }
                break;
            case kSlideStowingBucket1:
                telemetry.addData("Slide State: ", "STOWING 1");
                bucket_stow_move_step(kArmStowedPosition, SlideState.kSlideStowingBucket2);
                break;
            case kSlideStowingBucket2:
                telemetry.addData("Slide State: ", "STOWING 2");
                bucket_stow_wait_step (0.5, SlideState.kSlideStowingBucket3);
                break;
            case kSlideStowingBucket3:
                telemetry.addData("Slide State: ", "STOWING 3");
                bucket_stow_move_step(kArmTravelPosition, SlideState.kSlideStowingBucket4);
                break;
            case kSlideStowingBucket4:
                telemetry.addData("Slide State: ", "STOWING 4");
                bucket_stow_wait_step (0.5, SlideState.kSlideStowingBucket5);
                break;
            case kSlideStowingBucket5:
                telemetry.addData("Slide State: ", "STOWING 5");
                bucket_stow_move_step(kArmStowedPosition, SlideState.kSlideStowingBucket6);
                break;
            case kSlideStowingBucket6:
                telemetry.addData("Slide State: ", "STOWING 6");
                bucket_stow_wait_step (1, SlideState.kSlideStowingBucket7);
                break;
            case kSlideStowingBucket7:
                telemetry.addData("Slide State: ", "STOWING 7");
                if (!check_for_return_to_slide_ready()) {
                    if (slide_homed && gamepad1.a) {
                        slide_motor.setTargetPosition(kSlideRetracted2);
                        slide_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        slide_motor.setVelocity(1000);
                        slide_state = SlideState.kSlideStowingBucket8;
                    } else {
                        slide_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
                        slide_motor.setVelocity(0);
                    }
                }
                break;
            case kSlideStowingBucket8:
                telemetry.addData("Slide State: ", "STOWING 8");
                if (!check_for_return_to_slide_ready()) {
                    if (slide_homed && gamepad1.a) {
                        if (!slide_motor.isBusy()) { // completed movement
                            slide_state = SlideState.kIntakeReady;
                        }
                    }
                }
                break;
            case kSlideManualControl:
                telemetry.addData("Slide State: ", "MANUAL");
                slide_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODERS);
                if (gamepad1.dpad_up) {
                    slide_motor.setPower(.5);
                    // telemetry.addData("Slide V: ", slide_motor.getVelocity());
                } else if (gamepad1.dpad_down) {
                    slide_motor.setPower(-.2);
                } else if (slide_homed && (gamepad1.a || gamepad1.y)) {
                    slide_state = SlideState.kSlideAutomaticControl;
                } else {
                    slide_motor.setPower(0);
                    check_slide_homed();
                }
                break;
        }
    }

    private void check_slide_homed() {
        if (slide_at_bottom_limit()) {
            slide_homed = true;
            slide_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private void climb(){
        if(gamepad1.left_bumper && !gamepad1.right_bumper){ // deploy
            climber_motor.setPower(1);
        } else {
            climber_motor.setPower(-gamepad1.left_trigger);
        }
    }
    
    private void launch_airplane(){
        if(gamepad1.right_trigger > 0.1 && gamepad1.right_bumper){ // launch
            launcher_servo.setPosition(0.7);
        } else {
            launcher_servo.setPosition(0);
        }
    }
    

    private void drive() {
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
        
        
        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));
        
        telemetry.addData("Gyro", getHeading());
        
        double front_left_power = Py - Protate;
        double back_left_power = Px - Protate;
        double back_right_power = Py + Protate;
        double front_right_power = Px + Protate;
        front_left_wheel.setVelocity(front_left_power * Ri314PIDUtil.MaxVInTicks);
        back_left_wheel.setVelocity(back_left_power * Ri314PIDUtil.MaxVInTicks);
        back_right_wheel.setVelocity(back_right_power * Ri314PIDUtil.MaxVInTicks);
        front_right_wheel.setVelocity(front_right_power * Ri314PIDUtil.MaxVInTicks);

        // report velicity as a fraction
        // telemetry.addData("lf V", front_left_wheel.getVelocity() / Ri314PIDUtil.MaxVInTicks );
        // telemetry.addData("rf V", front_right_wheel.getVelocity() / Ri314PIDUtil.MaxVInTicks);
        // telemetry.addData("lr V", back_left_wheel.getVelocity() / Ri314PIDUtil.MaxVInTicks);
        // telemetry.addData("rr V", back_right_wheel.getVelocity() / Ri314PIDUtil.MaxVInTicks);
    }
    
    private void resetAngle(){
        if(gamepad1.right_bumper && gamepad1.left_bumper){
            reset_angle = getHeading() + reset_angle;
        }
    }
    
    private double getHeading(){
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
    
