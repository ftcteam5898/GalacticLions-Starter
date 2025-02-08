package org.firstinspires.ftc.team26248;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp(name="TeleOp NEWARM", group="Starter Code")
public class Test_TeleOP_Arm_refactor extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, armMotor, slideMotor;
    private Servo clawLeftMotor, clawRightMotor, wristServo;
    private IMU imu;
    private final double CLAW_LEFT_OPEN = 0.4;
    private final double CLAW_RIGHT_OPEN = 0.4;
    private final double CLAW_LEFT_CLOSE = 0.6;
    private final double CLAW_RIGHT_CLOSE = 0.6;
    private final double ARM_POWER_SCALING = 0.6; // Scaled power for joystick control
    private final double WRIST_DOWN = 0.22;
    private final double WRIST_MIDDLE = 0.11;
    private final double WRIST_UP = 0;
    final int TILT_HIGH = 2250;
    final int TILT_MEDIUM = 720;
    final int TILT_LOW = 480;
    private final double ARM_TICKS_TO_DEGREES = 0.0529;
    private final double SLIDE_TICKS_TO_INCHES = -0.01191;
    private final double ARM_LENGTH = 44.8; // unit: inches
    private final double MAX_ARM_ANGLE = 120.0; // Maximum arm rotation in degrees
    private final double MIN_ARM_ANGLE = 0.0;   // Minimum arm rotation in degrees
    private final double ARM_POWER = 0.6; // Power level for arm movement
    private final int MAX_SLIDE_TICKS = -2400; // Maximum slide extension ticks
    private final int MIN_SLIDE_TICKS = 0;     // Minimum slide retracted ticks
    public void updateslidelimit(){
        double slideLengthInches = slideMotor.getCurrentPosition() * SLIDE_TICKS_TO_INCHES;
        double armAngleDegrees = armMotor.getCurrentPosition() * ARM_TICKS_TO_DEGREES;
        armAngleDegrees = Math.max(MIN_ARM_ANGLE, Math.min(MAX_ARM_ANGLE, armAngleDegrees)); // Limit arm angle
        double armAngleRadians = Math.toRadians(armAngleDegrees);

        double armX = ARM_LENGTH * Math.cos(armAngleRadians);
        double armY = ARM_LENGTH * Math.sin(armAngleRadians);

        telemetry.addData("Updated Slide Length (inches)", slideLengthInches);
        telemetry.addData("Updated Arm X (inches)", armX);
        telemetry.addData("Updated Arm Y (inches)", armY);
    }



    // An Enum is used to represent arm states.
    // (This is one thing enums are designed to do)


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFrontLeft = hardwareMap.dcMotor.get("fl");
        motorBackLeft = hardwareMap.dcMotor.get("bl");
        motorFrontRight = hardwareMap.dcMotor.get("fr");
        motorBackRight = hardwareMap.dcMotor.get("br");

        // These are the extra moving parts
        armMotor = hardwareMap.dcMotor.get("arm");
        slideMotor = hardwareMap.dcMotor.get("slide");
        clawLeftMotor = hardwareMap.servo.get("vr");
        clawRightMotor = hardwareMap.servo.get("vl");
        wristServo = hardwareMap.servo.get("wr");





        // Reverse the left side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.REVERSE);


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        armMotor.setTargetPosition(TILT_LOW);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        imu.resetYaw();
        armMotor.setTargetPosition(TILT_LOW);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawLeftMotor.setPosition(.5);
        clawRightMotor.setPosition(.5);
        wristServo.setPosition(WRIST_MIDDLE);

    }
    public void updateSlideControl() {
        int slideTicks = slideMotor.getCurrentPosition();
        double slidePower = -gamepad2.left_stick_y;

        if ((slideTicks >= MAX_SLIDE_TICKS && slidePower < 0) || (slideTicks <= MIN_SLIDE_TICKS && slidePower > 0)) {
            slideMotor.setPower(slidePower);
        } else {
            slideMotor.setPower(0);
        }
    }
    public void updateArmControl() {
        int currentTicks = armMotor.getCurrentPosition();
        double currentDegrees = currentTicks * ARM_TICKS_TO_DEGREES;
        double armPower = -gamepad2.right_stick_y * ARM_POWER_SCALING; // Use right joystick for arm control

        // Prevent movement beyond limits
        if ((currentDegrees >= MAX_ARM_ANGLE && armPower > 0) || (currentDegrees <= MIN_ARM_ANGLE && armPower < 0)) {
            armMotor.setPower(0);
        } else {
            armMotor.setPower(armPower);
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {


        updateslidelimit();
        updateSlideControl();
        updateArmControl();
        // Compute slide extension length
        double slideLengthInches = slideMotor.getCurrentPosition() * SLIDE_TICKS_TO_INCHES;

        // Compute arm angle and endpoint position
        double armAngleDegrees = armMotor.getCurrentPosition() * ARM_TICKS_TO_DEGREES;
        armAngleDegrees = Math.max(MIN_ARM_ANGLE, Math.min(MAX_ARM_ANGLE, armAngleDegrees)); // Limit arm angle
        double armAngleRadians = Math.toRadians(armAngleDegrees);

        double armX = ARM_LENGTH * Math.cos(armAngleRadians); // Horizontal distance
        double armY = ARM_LENGTH * Math.sin(armAngleRadians); // Vertical height

        telemetry.addData("Slide Length (inches)", slideLengthInches);
        telemetry.addData("Arm Angle (degrees)", armAngleDegrees);
        telemetry.addData("Arm X (horizontal, inches)", armX);
        telemetry.addData("Arm Y (vertical, inches)", armY);
        telemetry.update();
        armMotor.setPower(1);






        // Drive Code
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers or options on PS4-style controllers.
        if (gamepad1.guide) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        int slidePos = slideMotor.getCurrentPosition();
        int tiltPos = armMotor.getCurrentPosition();
        telemetry.addData("Current slide position", slidePos);
        telemetry.addData("Current tilt position", tiltPos);

        // slide control

        

        if (gamepad2.a) {
            //open
            clawLeftMotor.setPosition(CLAW_LEFT_CLOSE);
            clawRightMotor.setPosition(CLAW_RIGHT_OPEN);
        }
        else if (gamepad2.b) {
            clawLeftMotor.setPosition(CLAW_LEFT_OPEN);
            clawRightMotor.setPosition(CLAW_RIGHT_CLOSE);
        }

        if (gamepad2.dpad_up){
            wristServo.setPosition(WRIST_UP);
        }
        else if (gamepad2.dpad_down){
            wristServo.setPosition(WRIST_DOWN);
        } else if (gamepad2.dpad_left) {
            wristServo.setPosition(WRIST_MIDDLE);
        }
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
