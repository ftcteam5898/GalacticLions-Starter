package org.firstinspires.ftc.team18443;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This particular OpMode executes a POV Teleop for a mecanum robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back and strafes left & right.
 * The Right stick rotates the robot left and right.
 *
 */

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
        Servo claw = hardwareMap.get(Servo.class, "claw");

        // Reverse one side of the motors
        // If it goes in reverse, reverse the other side
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define and Initialize the IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        imu.resetYaw();

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

            if (gamepad1.guide) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double X = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double Y = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(Y) + Math.abs(X) + Math.abs(rx), 1);
            double frontLeftPower = (Y + X + rx) / denominator;
            double backLeftPower = (Y - X + rx) / denominator;
            double frontRightPower = (Y - X - rx) / denominator;
            double backRightPower = (Y + X - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            //---------- Driver 2 ----------//

            int armPosition = arm.getCurrentPosition();

            if (gamepad2.dpad_up && armPosition >= -2700) { // Arm Up
                arm.setPower(-1);
            }
            else if (gamepad2.left_bumper) { // Arm Up - Sticky
                arm.setTargetPosition(-2700);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
            else if (gamepad2.dpad_down && armPosition <= 0) { // Arm Down
                arm.setPower(1);
            }
            else if (gamepad2.right_bumper) { // Arm Down - Sticky
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
            else {
                arm.setPower(0);
            }

            if (gamepad2.x) {
                claw.setPosition(0.5); // open
            }
            else if (gamepad2.b) {
                claw.setPosition(0.65); // close
            }
        }
    }
}
