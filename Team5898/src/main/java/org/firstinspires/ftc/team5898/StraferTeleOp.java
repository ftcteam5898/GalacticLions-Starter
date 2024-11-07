package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This particular OpMode executes a POV Teleop for a mecanum robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back and strafes left & right.
 * The Right stick rotates the robot left and right.
 *
 */

@TeleOp(name="Strafer Tele Op", group="Starter Code")
public class StraferTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("RL");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("RR");
        // These are the extra moving parts
        DcMotor motorArmTilt = hardwareMap.dcMotor.get("Arm");
        DcMotor motorBeltDrive = hardwareMap.dcMotor.get("Belt");
        Servo servoClaw = hardwareMap.servo.get("Claw");
        Servo servoWrist = hardwareMap.servo.get("Wrist");
        double wristPos = 1.0;
        servoWrist.setPosition(wristPos);
        sleep(1000);
        double CLAW_CLOSE = 0.27;
        servoClaw.setPosition(CLAW_CLOSE);//close

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
       // motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBeltDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmTilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
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

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            int slidePos = motorBeltDrive.getCurrentPosition();
            int tiltPos = motorArmTilt.getCurrentPosition();
            telemetry.addData("Current slide position", slidePos);
            telemetry.addData("Current tilt position", tiltPos);
            telemetry.update();

            if (gamepad2.dpad_up && slidePos <= 2300)
            {
                motorBeltDrive.setPower(.5);
            }
            else if (gamepad2.dpad_down && slidePos >= 30)
            {
                motorBeltDrive.setPower(-.5);
            }
            else
            {
                motorBeltDrive.setPower(0);
            }

            if (gamepad2.dpad_left)
            {
                motorArmTilt.setPower(-.4);
            }
            else if (gamepad2.dpad_right)
            {
                motorArmTilt.setPower(.4);
            }
            else if (!gamepad2.dpad_left && !gamepad2.dpad_right){
                motorArmTilt.setPower(0);
            }

            if (gamepad2.left_bumper)
            {
                motorArmTilt.setTargetPosition(540);
                motorArmTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorArmTilt.setPower(.5);
                while (motorArmTilt.isBusy()){
                    telemetry.addData("Busy...", "");
                    telemetry.update();
                }
                motorArmTilt.setPower(0);
            }
            else if (gamepad2.right_bumper)
            {
                motorArmTilt.setTargetPosition(100);
                motorArmTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorArmTilt.setPower(.5);
                while (motorArmTilt.isBusy()){
                    telemetry.addData("Busy...", "");
                    telemetry.update();
                }
                motorArmTilt.setPower(0);
            }

            if (gamepad2.y)
            {
                wristPos -= .001;
                servoWrist.setPosition(wristPos);
            }
            else if (gamepad2.a)
            {
                wristPos += .001;
                servoWrist.setPosition(wristPos);
            }

            if (gamepad1.left_bumper)
            {
                servoClaw.setPosition(.4);//open
            }
            else if (gamepad1.right_bumper)
            {
                servoClaw.setPosition(CLAW_CLOSE);//close
            }


        }
    }
}
