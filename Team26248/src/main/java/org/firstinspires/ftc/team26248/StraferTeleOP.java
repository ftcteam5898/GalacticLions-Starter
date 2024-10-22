package org.firstinspires.ftc.team26248;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp V4", group="TeleOp")
public class StraferTeleOP extends LinearOpMode{
    @Override
    public void runOpMode() {
        //Get the Motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("fr");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("bl");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("br");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        Servo clawLeftMotor = hardwareMap.servo.get("vl");
        Servo clawRightMotor = hardwareMap.servo.get("vr");

        //Set the Direction of the Motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRightMotor.setDirection(Servo.Direction.REVERSE);
        clawLeftMotor.setDirection(Servo.Direction.FORWARD);
        waitForStart();

        if(isStopRequested()) return;

        // Variables for smooth movement
        double prevFrontLeftPower = 0;
        double prevFrontRightPower = 0;
        double prevBackLeftPower = 0;
        double prevBackRightPower = 0;
        double smoothingFactor = 0.2; // Adjust for more or less smoothing
        boolean moveSwitch = false;

        //Run the OpMode
        while(opModeIsActive()) {
            //Get the Joystick Values
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //Arm Control
            double armPower = gamepad2.right_stick_y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double targetFrontLeftPower = (y + x + rx) / denominator * 0.925;
            double targetFrontRightPower = (y - x - rx) / denominator * 0.925;
            double targetBackLeftPower = (y - x + rx) / denominator * 0.925;
            double targetBackRightPower = (y + x - rx) / denominator * 0.925;

            // Apply smoothing to motor powers
            double frontLeftPower = prevFrontLeftPower + (targetFrontLeftPower - prevFrontLeftPower) * smoothingFactor;
            double frontRightPower = prevFrontRightPower + (targetFrontRightPower - prevFrontRightPower) * smoothingFactor;
            double backLeftPower = prevBackLeftPower + (targetBackLeftPower - prevBackLeftPower) * smoothingFactor;
            double backRightPower = prevBackRightPower + (targetBackRightPower - prevBackRightPower) * smoothingFactor;

            // Update previous power values
            prevFrontLeftPower = frontLeftPower;
            prevFrontRightPower = frontRightPower;
            prevBackLeftPower = backLeftPower;
            prevBackRightPower = backRightPower;

            // Claw Control with Position Limits
            double clawLeftPosition = clawLeftMotor.getPosition();
            double clawRightPosition = clawRightMotor.getPosition();

            if (gamepad2.a) {
                clawLeftMotor.setPosition(Math.min(clawLeftPosition + 0.1, 1.0));
                clawRightMotor.setPosition(Math.min(clawRightPosition + 0.1, 1.0));
            }
            else if (gamepad2.b) {
                clawLeftMotor.setPosition(Math.max(clawLeftPosition - 0.1, 0.0));
                clawRightMotor.setPosition(Math.max(clawRightPosition - 0.1, 0.0));
            }

            if(gamepad1.left_bumper||gamepad1.right_bumper){
                moveSwitch = !moveSwitch;
            }

            // SecuritySwitch
            if (gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1 || gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            } else {
                if (moveSwitch) {
                    motorFrontLeft.setPower(-backRightPower * 0.8);
                    motorFrontRight.setPower(-backLeftPower * 0.8);
                    motorBackLeft.setPower(-frontRightPower * 0.8);
                    motorBackRight.setPower(-frontLeftPower * 0.8);
                } else {
                    motorFrontLeft.setPower(frontLeftPower * 0.8);
                    motorFrontRight.setPower(frontRightPower * 0.8);
                    motorBackLeft.setPower(backLeftPower * 0.8);
                    motorBackRight.setPower(backRightPower * 0.8);
                }
                armMotor.setPower(armPower * 0.9);
            }
        }
    }
}
