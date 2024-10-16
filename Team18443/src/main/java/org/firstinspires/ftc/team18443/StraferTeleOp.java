package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name="Strafer_Controls", group="Tele")
public class StraferTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "lf");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "rf");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "lb");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "rb");

        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and Initialize Servos
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        // Reverse one side of the motors
        // If it goes in reverse, reverse the other side
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {

            //---------- Driver 1 ----------//

            // Driving controls
            double y = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);


            //---------- Driver 2 ----------//

            if (gamepad2.dpad_up) { // Arm Up
                arm.setPower(-1);
            }
            else if (gamepad2.dpad_down) { // Arm Down
                arm.setPower(1);
            }
            else {
                arm.setPower(0);
            }

            if (gamepad2.x) {
                claw.setPosition(0.2); // open
            }
            else if (gamepad2.b) {
                claw.setPosition(0); // close
            }

            if (gamepad2.y) {
                wrist.setPosition(1); // raise
            }
            else if (gamepad2.a) {
                wrist.setPosition(0.5); // lower
            }

        }
    }
}