package org.firstinspires.ftc.team26248;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="Test")
public class Servo_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        Servo rightServo = hardwareMap.servo.get("vr");
        Servo leftServo = hardwareMap.servo.get("vl");
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                rightServo.setPosition(rightServo.getPosition()+0.1);
                telemetry.update();
                sleep(500);
            } else if (gamepad1.b) {
                rightServo.setPosition(rightServo.getPosition()-0.1);
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_up) {
                leftServo.setPosition(leftServo.getPosition()+0.1);
                telemetry.update();
                sleep(500);
            } else if (gamepad1.dpad_down) {
                leftServo.setPosition(leftServo.getPosition()-0.1);
                telemetry.update();
                sleep(500);
            }


            telemetry.addData("ServoRight Position", rightServo.getPosition());
            telemetry.addData("ServoLeft Position", leftServo.getPosition());
            telemetry.update();
        }
    }
}