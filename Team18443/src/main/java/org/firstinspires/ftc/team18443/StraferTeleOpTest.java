package org.firstinspires.ftc.team18443;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * This particular OpMode executes a POV Teleop for a mecanum robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back and strafes left & right.
 * The Right stick rotates the robot left and right.
 *
 */
@TeleOp(name="Strafer Tele Op - Test Code", group="Starter Code")
public class StraferTeleOpTest extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft, backLeft, frontRight, backRight, arm;
    private Servo claw;
    private IMU imu;
    final double CLAW_OPEN = 0.5;
    final double CLAW_CLOSE = 0.65;

    final int ARM_LOW = 10;
    final int ARM_HIGH_CHAMBER = 2700;
    final int ARM_RELEASE = 2200;
    final int ARM_LOW_BASKET = 3000;


    // An Enum is used to represent arm states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_RELEASE,
        LIFT_RETRACT
    };

    LiftState liftState = LiftState.LIFT_START;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lb");
        backRight = hardwareMap.get(DcMotor.class, "rb");

        // These are the extra moving parts
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");

        // Reverse the left side motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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

        arm.setTargetPosition(ARM_LOW);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        arm.setPower(0.5);

        telemetry.addData("State: ", ""+liftState);


        switch (liftState) {
            case LIFT_START:
                // wait for input
                // b grab and go up
                // x drop and go down
                if (gamepad2.b) {
                    claw.setPosition(CLAW_CLOSE);
                    runtime.reset();
                    liftState = LiftState.LIFT_EXTEND;
                }
                if (gamepad2.x) {
                    arm.setTargetPosition(ARM_LOW);
                }
                break;
            case LIFT_EXTEND:
                // check if the claw has finished closing,
                if (runtime.seconds() > .5)
                {
                    arm.setTargetPosition(ARM_HIGH_CHAMBER);
                }

                if (gamepad2.x) {
                    arm.setTargetPosition(ARM_RELEASE);
                    liftState = LiftState.LIFT_RELEASE;
                    runtime.reset();
                }
                break;
            case LIFT_RELEASE:
                if (runtime.seconds() > 0.5)
                {
                    claw.setPosition(CLAW_OPEN);

                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (runtime.seconds() > 0.5)
                {
                    arm.setTargetPosition(ARM_LOW);
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                // should never be reached, as armState should never be null
                liftState = LiftState.LIFT_START;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad2.guide && liftState != LiftState.LIFT_START) {
            liftState = LiftState.LIFT_START;
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

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        if (gamepad2.left_bumper)
        {
            claw.setPosition(CLAW_CLOSE);
        }
        else if (gamepad2.right_bumper) {
            claw.setPosition(CLAW_OPEN);
        }

        telemetry.addData("Arm Position: ", arm.getCurrentPosition());
        telemetry.addData("Target Position: ", arm.getTargetPosition());
        telemetry.addData("Mode: ", arm.getMode());
        if (gamepad2.dpad_up && arm.getCurrentPosition() < 2600)
        {
            telemetry.addLine("Going up");
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(1);
            arm.setTargetPosition(arm.getCurrentPosition());
        }
        else if (gamepad2.dpad_down && arm.getCurrentPosition() > 0)
        {
            telemetry.addLine("Going down");
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(-1);
            //arm.setTargetPosition(arm.getCurrentPosition()-20);
        }
        else if (!gamepad2.dpad_up && !gamepad2.dpad_down)
        {

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
