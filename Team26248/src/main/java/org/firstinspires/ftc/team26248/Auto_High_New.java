/*
    @Author: HackingU0
 */


package org.firstinspires.ftc.team26248;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AutoHigh (Use this,No Parking)",group = "Autonomous")
public class Auto_High_New extends LinearOpMode {
    DcMotor frontLeft,frontRight, backLeft, backRight, armMotor, slideMotor;
    Servo clawLeft, clawRight;

    double cpr = 537.7; //312 RPM Gobilda
    double gearRatio = 1;
    double diameter = 3.779; //for Gobilda 104mm Mecanum wheel
    double cpi = (cpr*gearRatio) / (Math.PI * diameter);
    double bias = 0.94;
    double strafeBias = 1.0;
    double conversion = cpi * bias;
    Claw claw;
    Slide slide;
    Arm arm;
    IMU imu;
    public enum Slide_Position {
        DOWN,
        MID,
        UP
    }


    public class Claw {
        private Servo clawLeft;
        private Servo clawRight;
        private final double CLAW_LEFT_OPEN = 0.3;
        private final double CLAW_RIGHT_OPEN = 0.7;
        private final double CLAW_LEFT_CLOSE = 0.75;
        private final double CLAW_RIGHT_CLOSE = 0.25;
        public Claw(Servo clawLeft, Servo clawRight) {
            this.clawLeft = clawLeft;
            this.clawRight = clawRight;
        }
        public void open() {
            clawLeft.setPosition(CLAW_LEFT_OPEN);
            clawRight.setPosition(CLAW_RIGHT_OPEN);
        }
        public void close() {
            clawLeft.setPosition(CLAW_LEFT_CLOSE);
            clawRight.setPosition(CLAW_RIGHT_CLOSE);
        }

    }


    public class Slide{


        private final DcMotor slideMotor;



        private final int POSITION_DOWN = 0;
        private final int POSITION_MID = -1540;
        private final int POSITION_UP = -3050;

        public Slide(DcMotor slideMotor) {
            this.slideMotor = slideMotor;
            this.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public void expandToPosition(Slide_Position position) {
            int targetPosition = 0;


            switch (position) {
                case DOWN:
                    targetPosition = POSITION_DOWN;
                    break;
                case MID:
                    targetPosition = POSITION_MID;
                    break;
                case UP:
                    targetPosition = POSITION_UP;
                    break;
            }

            slideMotor.setTargetPosition(targetPosition);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(1);
        }

        public void contract() {
            expandToPosition(Slide_Position.DOWN);
        }

        public void expandDown() {
            expandToPosition(Slide_Position.MID);
        }

        public void expandUP() {
            expandToPosition(Slide_Position.UP);
        }

    }


    public class Arm{
        private DcMotor armMotor;

        //Change Arm Status Here
        private final int arm_up = 2100;
        private final int arm_down = 430;

        public Arm(DcMotor armMotor) {
            this.armMotor = armMotor;
            this.armMotor.setPower(.5);
            this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public void up(){
            armMotor.setTargetPosition(arm_up);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public void down(){
            armMotor.setTargetPosition(arm_down);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.5);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }

    public void setMotorMode(DcMotor.RunMode mode){
        armMotor.setMode(mode);
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void waitForMotors() {
        long startTime = System.currentTimeMillis();
        long timeout = 3250;

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy() ||
                        armMotor.isBusy() || slideMotor.isBusy())) {

            if (System.currentTimeMillis() - startTime > timeout) {
                telemetry.addData("Warning", "Motor timeout!");
                telemetry.update();
                break;
            }

            telemetry.addData("Status", "Motors Running...");
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        inithardware();
        waitForStart();

        strafeLeft(12,0.5);
        arm.down();
        slide.expandDown();
        forward(13.5,0.6);
        waitForMotors();
        claw.close();
        waitForMotors();
        slide.contract();
        waitForMotors();
        turnLeft(-90,0.6);
        waitForMotors();
        turnLeft(-40,0.6);
        waitForMotors();
        forward(9,0.5);
        waitForMotors();
        arm.up();
        waitForMotors();
        slide.expandUP();
        waitForMotors();
        claw.open();
        waitForMotors();
        armMotor.setTargetPosition(armMotor.getCurrentPosition() - 30);
        waitForMotors();
        back(2,0.5);
        waitForMotors();
        armMotor.setTargetPosition(armMotor.getCurrentPosition() + 30);
        waitForMotors();
        slide.contract();
        waitForMotors();
        arm.down();
        waitForMotors();
        turnRight(-40,0.7);
        waitForMotors();
        turnRight(-90,0.7);
        waitForMotors();
        back(5,0.6);
        waitForMotors();
        stop();











        //Do not touch
        while(opModeIsActive()){
            if(!armMotor.isBusy()){
                armMotor.setPower(1);
            }
            if(!slideMotor.isBusy()){
                slideMotor.setPower(1);
            }

            telemetry.addData("Current Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
            telemetry.update();
        }
        waitForMotors();
    }
    private void inithardware(){
        clawLeft = hardwareMap.servo.get("vl");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        clawRight = hardwareMap.servo.get("vr");

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slide");
        armMotor = hardwareMap.dcMotor.get("arm");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = new Claw(clawLeft,clawRight);
        slide = new Slide(slideMotor);
        arm = new Arm(armMotor);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Front Left", frontLeft != null ? "Initialized" : "Not Initialized");
        telemetry.addData("Front Right", frontRight != null ? "Initialized" : "Not Initialized");
        telemetry.addData("Back Left", backLeft != null ? "Initialized" : "Not Initialized");
        telemetry.addData("Back Right", backRight != null ? "Initialized" : "Not Initialized");
        telemetry.addData("Arm Motor", armMotor != null ? "Initialized" : "Not Initialized");
        telemetry.addData("Slide Motor", slideMotor != null ? "Initialized" : "Not Initialized");
        telemetry.update();

        initGyro();
        arm.down();
        slide.contract();
        claw.open();
    }
    public void forward(double inches, double speed) {
        moveToPosition(inches, speed);
    }

    /**
     * Used to make the robot move backward by a specified number of inches.
     *
     * @param inches The distance to move backward, in inches
     * @param speed Speed range is [0,1]
     */
    public void back(double inches, double speed) {
        moveToPosition(-inches, speed);
    }

    /**
     * Make the robot rotate left.
     *
     * @param degrees The angle to rotate
     * @param speed Speed range is [0,1]
     */
    public void turnLeft(double degrees, double speed) {
        turnWithGyro(degrees, -speed);
    }

    /**
     * Make the robot rotate to the right.
     * @param degrees The angle to rotate
     * @param speed Speed range is [0,1]
     */
    public void turnRight(double degrees, double speed) {
        turnWithGyro(degrees, speed);
    }


    public void strafeLeft(double inches, double speed) {
        strafeToPosition(-inches, speed);
    }


    public void strafeRight(double inches, double speed) {
        strafeToPosition(inches, speed);
    }


    public void moveToPosition(double inches, double speed){
        int move = (int)(Math.round(inches*conversion));
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Busy...", "");
            telemetry.update();
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void turnWithGyro(double degrees, double speedDirection) {

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();



        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();

        double first;
        double second;


        if (speedDirection > 0) {
            if (degrees > 10) {
                first = (degrees - 10) + devertify(yaw);
            } else {
                first = devertify(yaw);
            }
            second = degrees + devertify(yaw);
        }


        else {
            if (degrees > 10) {
                first = devertify(-(degrees - 10) + devertify(yaw));
            } else {
                first = devertify(yaw);
            }
            second = devertify(-degrees + devertify(yaw));
        }


        double firsta = convertify(first - 5);
        double firstb = convertify(first + 5);
        turnWithEncoder(speedDirection);

        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        } else {
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {// 是否在范围内？
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }

        double seconda = convertify(second - 5);// 175
        double secondb = convertify(second + 5);// -175
        turnWithEncoder(speedDirection / 3);

        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {// 是否在范围内？
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // 是否应为负数？
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {// 是否在范围内？
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // 是否应为负数？
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void strafeToPosition(double inches, double speed) {
        int move = (int) (Math.round(inches * cpi * strafeBias));
        int targetBackLeft = backLeft.getCurrentPosition() - move;
        int targetFrontLeft = frontLeft.getCurrentPosition() + move;
        int targetBackRight = backRight.getCurrentPosition() + move;
        int targetFrontRight = frontRight.getCurrentPosition() - move;

        backLeft.setTargetPosition(targetBackLeft);
        frontLeft.setTargetPosition(targetFrontLeft);
        backRight.setTargetPosition(targetBackRight);
        frontRight.setTargetPosition(targetFrontRight);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Strafing to position", "In Progress");
            telemetry.update();
        }

        // Stop all motors
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 360) {
            degrees = degrees - 360;
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 179) {
            degrees = -(360 - degrees);
        }
        return degrees;
    }

    /*
     * Initialize Gyro
     */
    public void initGyro() {
        // ftc-docs.firstinspires.org
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void turnWithEncoder(double input) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontLeft.setPower(input);
        backLeft.setPower(input);
        frontRight.setPower(-input);
        backRight.setPower(-input);
    }
}
