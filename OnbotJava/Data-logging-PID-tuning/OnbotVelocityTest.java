package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "OnbotVelocityTest", group = "Station")
public class OnbotVelocityTest extends LinearOpMode {

  private OpticalDistanceSensor distance;
  private Servo stdservo;
  private DcMotorEx motorL;
  private DcMotorEx motorR;
  private CRServo crservo;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double MaxV;
    double currentV;
    ElapsedTime timer;

    distance = hardwareMap.opticalDistanceSensor.get("distance");
    stdservo = hardwareMap.servo.get("stdservo");
    motorL = hardwareMap.get(DcMotorEx.class, "motorL");
    motorR = hardwareMap.get(DcMotorEx.class, "motorR");
    crservo = hardwareMap.crservo.get("crservo");

    idle();
    // Put initialization blocks here.
    distance.enableLed(true);
    stdservo.scaleRange(0.1, 0.9);
    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorL.setDirection(DcMotorSimple.Direction.REVERSE);
    MaxV = 0;
    timer = new ElapsedTime();
    // Put run blocks here.
    waitForStart();
    timer.reset();
    crservo.setPower(1);
    stdservo.setPosition(0);
    sleep(1000);
    stdservo.setPosition(1);
    sleep(1000);
    stdservo.setPosition(0);
    crservo.setPower(0);
    while (opModeIsActive() && distance.getLightDetected() < 0.22) {
      // Tests show max V is 3420
      motorL.setVelocity(0.3*3420, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
      motorR.setVelocity(0.3*3420, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
      if (timer.time() > 5) {
        currentV = ((DcMotorEx) motorL).getVelocity(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        if (currentV > MaxV) {
          MaxV = currentV;
        }
        currentV = ((DcMotorEx) motorR).getVelocity(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        if (currentV > MaxV) {
          MaxV = currentV;
        }
      }
      telemetry.addData("range", distance.getLightDetected());
      telemetry.addData("L position", motorL.getCurrentPosition());
      telemetry.addData("R position", motorR.getCurrentPosition());
      telemetry.addData("L V", ((DcMotorEx) motorL).getVelocity(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
      telemetry.addData("R V", ((DcMotorEx) motorR).getVelocity(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
      telemetry.addData("Max V", MaxV);
      telemetry.update();
    }
    motorL.setPower(0);
    motorR.setPower(0);
    while (opModeIsActive()) {
    }
  }
}
