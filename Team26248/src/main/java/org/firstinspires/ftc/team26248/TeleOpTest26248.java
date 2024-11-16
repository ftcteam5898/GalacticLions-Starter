package org.firstinspires.ftc.team26248;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Tele Op - Test Code - 26248", group="Starter Code")
public class TeleOpTest26248 extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, armMotor, slideMotor;
    private Servo clawLeftMotor, clawRightMotor;
    private IMU imu;
    private final double CLAW_LEFT_OPEN = 0.25;
    private final double CLAW_RIGHT_OPEN = 0.5;
    private final double CLAW_LEFT_CLOSE = 0.5;
    private final double CLAW_RIGHT_CLOSE = 0.75;
    final int TILT_HIGH = 2000;
    final int TILT_LOW = 500;

    private boolean slideLimit;


    // An Enum is used to represent arm states.
    // (This is one thing enums are designed to do)
    public enum ArmState {
        ARM_DOWN,
        ARM_UP,
    };

    ArmState armState = ArmState.ARM_DOWN;

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


        slideLimit = true;

        // Reverse the left side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        clawLeftMotor.setPosition(CLAW_LEFT_OPEN);
        clawRightMotor.setPosition(CLAW_RIGHT_OPEN);

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        armMotor.setPower(0.5);
        telemetry.addData("State: ", ""+armState);
        telemetry.addData("Slide Limited? ", slideLimit);


        switch (armState) {
            case ARM_DOWN:
                // wait for input
                if (gamepad2.left_bumper) {
                    armMotor.setTargetPosition(TILT_HIGH);
                    armState = ArmState.ARM_UP;
                    slideLimit = false;
                }
                break;
            case ARM_UP:
                // check if the arm has finished tilting,
                // otherwise do nothing.
                if (gamepad2.right_bumper) {
                    armMotor.setTargetPosition(TILT_LOW);
                    armState = ArmState.ARM_DOWN;
                    slideLimit = true;
                }
                break;
            default:
                // should never be reached, as armState should never be null
                armState = ArmState.ARM_DOWN;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad2.guide && armState != ArmState.ARM_DOWN) {
            armState = ArmState.ARM_DOWN;
        }


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
        if (slideLimit)
        {
            if (slidePos > -1300 && gamepad2.left_stick_y < 0)
                slideMotor.setPower(gamepad2.left_stick_y * .5);
            else if (slidePos < -50 && gamepad2.left_stick_y > 0)
                slideMotor.setPower(gamepad2.left_stick_y * .5);
            else
            {
                slideMotor.setPower(0);
            }

        }
        else{
            slideMotor.setPower(gamepad2.left_stick_y * .5);
        }

        if (gamepad2.a) {
            //open
            clawLeftMotor.setPosition(CLAW_LEFT_CLOSE);
            clawRightMotor.setPosition(CLAW_RIGHT_OPEN);
        }
        else if (gamepad2.b) {
            clawLeftMotor.setPosition(CLAW_LEFT_OPEN);
            clawRightMotor.setPosition(CLAW_RIGHT_CLOSE);
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
