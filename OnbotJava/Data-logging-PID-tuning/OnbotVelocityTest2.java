package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "OnbotVelocityTest2", group = "Station")
public class OnbotVelocityTest2 extends LinearOpMode {
  private DcMotorEx motorL;

  @Override
  public void runOpMode() {
    motorL = hardwareMap.get(DcMotorEx.class, "motorL");
    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    waitForStart();
    while (opModeIsActive()) {
      motorL.setVelocity(0.3*3420);
    }
  }
}