package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Gamma Strafer Tele Op - Test Code", group="Gamma Bot")
public class Gamma_StraferTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private final double INTAKE_IN_LEFT = .78;
    private final double INTAKE_IN_RIGHT = .22;

    private final double CLAW_OPEN = 0.4;
    private final double CLAW_CLOSE = 0.27;
    private double wristPos;
    private RobotHardware robot;

    public enum BotState {
        NEUTRAL,
        INTAKE_OUT,
        GRAB_DOWN,
        INTAKE_GRAB_IN,
        OUTTAKE_GRAB_FLIP,
        OUTTAKE_UP,
        OUTTAKE_RELEASE_DOWN
    };

    BotState botState = BotState.NEUTRAL;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();  // Initialize all hardware

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
    }

    @Override
    public void loop() {


        switch (botState) {
            case NEUTRAL:
                robot.rightIntake.setPosition(INTAKE_IN_RIGHT);
                robot.leftIntake.setPosition(INTAKE_IN_LEFT);
                robot.wrist.setPosition(.3);
                robot.grabber.setPosition(.2); //closed


                // wait for input
                if (gamepad2.dpad_left) {
                    //change state to INTAKE_OUT
                    botState = BotState.INTAKE_OUT;
                }
                break;
            case INTAKE_OUT:
                // set intake servos to go out
                robot.leftIntake.setPosition(1);
                robot.rightIntake.setPosition(0);
                robot.wrist.setPosition(.9);
                robot.grabber.setPosition(0); //open
                // wait for input
                if (gamepad1.x) {
                    //change state to GRAB_DOWN
                    botState = BotState.GRAB_DOWN;
                    runtime.reset();
                }
                else if (gamepad2.dpad_right) {
                    //change state to INTAKE_OUT
                    botState = BotState.NEUTRAL;
                }
                break;
            case GRAB_DOWN:
                // wrist goes down, grabber closes, then wrist comes up, and intake comes in
                if (runtime.seconds() < .5)
                {
                    robot.wrist.setPosition(1);
                    robot.grabber.setPosition(.2);
                } else if (runtime.seconds() > .5) {
                    robot.wrist.setPosition(0);
                    robot.leftIntake.setPosition(INTAKE_IN_LEFT);
                    robot.rightIntake.setPosition(INTAKE_IN_RIGHT);
                    botState = BotState.INTAKE_GRAB_IN;
                }
                break;
            case INTAKE_GRAB_IN:
                //wait for input
                break;

            default:
                botState = BotState.NEUTRAL;
        }






        // Drive Code
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        if (gamepad1.right_bumper)
        {
            y = clamp(y, -.25, .25);
            x = clamp(x, -.25, .25);
        }
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers or options on PS4-style controllers.
        if (gamepad1.guide) {
            robot.imu.resetYaw();
        }

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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


        robot.leftFront.setPower(frontLeftPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.rightRear.setPower(backRightPower);




        telemetry.update();
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }



}