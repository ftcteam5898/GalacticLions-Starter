package org.firstinspires.ftc.team26248;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="Motor Debug Test", group="Debug")
public class MotorDebug extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Get the Motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("fr");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("bl");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("br");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Motor Front Left Test
            if (gamepad1.a) {
                // Press 'A' to run motorFrontLeft forward
                motorFrontLeft.setPower(0.5);
                telemetry.addData("Motor Front Left", "Running FORWARD");
            } else if (gamepad1.b) {
                // Press 'B' to run motorFrontLeft in reverse
                motorFrontLeft.setPower(-0.5);
                telemetry.addData("Motor Front Left", "Running REVERSE");
            } else {
                motorFrontLeft.setPower(0);
            }

            // Motor Front Right Test
            if (gamepad1.x) {
                // Press 'X' to run motorFrontRight forward
                motorFrontRight.setPower(0.5);
                telemetry.addData("Motor Front Right", "Running FORWARD");
            } else if (gamepad1.y) {
                // Press 'Y' to run motorFrontRight in reverse
                motorFrontRight.setPower(-0.5);
                telemetry.addData("Motor Front Right", "Running REVERSE");
            } else {
                motorFrontRight.setPower(0);
            }

            // Motor Back Left Test
            if (gamepad1.dpad_up) {
                // Press 'DPAD_UP' to run motorBackLeft forward
                motorBackLeft.setPower(0.5);
                telemetry.addData("Motor Back Left", "Running FORWARD");
            } else if (gamepad1.dpad_down) {
                // Press 'DPAD_DOWN' to run motorBackLeft in reverse
                motorBackLeft.setPower(-0.5);
                telemetry.addData("Motor Back Left", "Running REVERSE");
            } else {
                motorBackLeft.setPower(0);
            }

            // Motor Back Right Test
            if (gamepad1.dpad_left) {
                // Press 'DPAD_LEFT' to run motorBackRight forward
                motorBackRight.setPower(0.5);
                telemetry.addData("Motor Back Right", "Running FORWARD");
            } else if (gamepad1.dpad_right) {
                // Press 'DPAD_RIGHT' to run motorBackRight in reverse
                motorBackRight.setPower(-0.5);
                telemetry.addData("Motor Back Right", "Running REVERSE");
            } else {
                motorBackRight.setPower(0);
            }

            telemetry.update();
        }
    }
}

/*
 * Operation Instructions:
 * 1. Press 'A' to test motorFrontLeft forward
 * 2. Press 'B' to test motorFrontLeft reverse
 * 3. Press 'X' to test motorFrontRight forward
 * 4. Press 'Y' to test motorFrontRight reverse
 * 5. Press 'DPAD_UP' to test motorBackLeft forward
 * 6. Press 'DPAD_DOWN' to test motorBackLeft reverse
 * 7. Press 'DPAD_LEFT' to test motorBackRight forward
 * 8. Press 'DPAD_RIGHT' to test motorBackRight reverse
 */