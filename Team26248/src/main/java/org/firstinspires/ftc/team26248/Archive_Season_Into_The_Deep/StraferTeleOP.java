package org.firstinspires.ftc.team26248.Archive_Season_Into_The_Deep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="TeleOp V5", group="TeleOp")
public class StraferTeleOP extends LinearOpMode{
    @Override
    public void runOpMode() {
        //Get the Motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("fr");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("bl");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("br");

        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        DcMotor slideMotor = hardwareMap.dcMotor.get("slide");
        Servo clawLeftMotor = hardwareMap.servo.get("vl");
        Servo clawRightMotor = hardwareMap.servo.get("vr");

        //Set the Direction of the Motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRightMotor.setDirection(Servo.Direction.REVERSE);
        clawLeftMotor.setDirection(Servo.Direction.FORWARD);
//        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            double slidePower = gamepad2.left_stick_y;

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



            if (gamepad2.a) {
                //open
                clawLeftMotor.setPosition(0.25);
                clawRightMotor.setPosition(0.5);
            }
            else if (gamepad2.b) {
                clawLeftMotor.setPosition(0.5);
                clawRightMotor.setPosition(0.75);
            }


            if(gamepad1.a){
                moveSwitch = !moveSwitch;
            }

            // SafetySwitch
            if (gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1 || gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                slideMotor.setPower(0);
            } else {
                if (moveSwitch) {
                    motorFrontLeft.setPower(-backRightPower * 0.85);
                    motorFrontRight.setPower(-backLeftPower * 0.85);
                    motorBackLeft.setPower(-frontRightPower * 0.85);
                    motorBackRight.setPower(-frontLeftPower * 0.85);

                } else {
                    motorFrontLeft.setPower(frontLeftPower * 0.85);
                    motorFrontRight.setPower(frontRightPower * 0.85);
                    motorBackLeft.setPower(backLeftPower * 0.85);
                    motorBackRight.setPower(backRightPower * 0.85);
                }
                armMotor.setPower(armPower * 0.9);
                if(armMotor.getCurrentPosition()>=2600){
                    if(slideMotor.getCurrentPosition()>=1700){
                        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        slideMotor.setPower(0);
                    } else if (slideMotor.getCurrentPosition()<1700) {
                        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        slideMotor.setPower(slidePower * 0.9);

                    }
                }
                else if(armMotor.getCurrentPosition()<2600||armMotor.getCurrentPosition()>1300){
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    slideMotor.setPower(slidePower * 0.9);
                }
                if(armPower==0){
                    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }
    }
}
