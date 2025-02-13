/*
    @Author: HackingU0
 */


package org.firstinspires.ftc.team26248;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous(name = "AutoHigh (02/12/2025 Update)",group = "Autonomous")
public class Auto_High_New extends LinearOpMode {
    DcMotor frontLeft,frontRight, backLeft, backRight, armMotor, slideMotor;
    Servo clawLeft, clawRight, wristServo;

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



    public class Claw {
        private Servo clawLeft;
        private Servo clawRight;
        private final double CLAW_LEFT_CLOSE = 0.6; //TODO: The Value
        private final double CLAW_RIGHT_CLOSE = 0.4;//TODO: The Value
        private final double CLAW_LEFT_OPEN = 0.4;//TODO: The Value
        private final double CLAW_RIGHT_OPEN = 0.6;//TODO: The Value
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
    public class Wrist {
        private Servo wristServo;

        public Wrist(Servo wristServo) {
            if (wristServo == null){
                throw new IllegalArgumentException("wristServo not showing up");
            }
            this.wristServo = wristServo;
        }

        public void flat(){
            wristServo.setPosition(0.11);
        }
        public void down(){
            wristServo.setPosition(0.22);
        }
    }

    public class Slide{
        private DcMotor slideMotor;

        public Slide(DcMotor slideMotor){
            this.slideMotor = slideMotor;
        }
        public void expandDown(){
            slideMotor.setTargetPosition(-870); //TODO: The Value
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(.7);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public void contract(){
            slideMotor.setTargetPosition(0);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(.7);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public void expandUP(){
            slideMotor.setTargetPosition(-2400); //TODO: The Value
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(.9);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

    }
    public class Arm{
        private DcMotor armMotor;

        //Change Arm Status Here
        private final int arm_up = 1945; //TODO: The Value
        private final int arm_down = 480;//TODO: The Value

        public Arm(DcMotor armMotor) {
            this.armMotor = armMotor;
            this.armMotor.setPower(1);
            this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public void up(){
            armMotor.setTargetPosition(arm_up);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public void down(){
            armMotor.setTargetPosition(arm_down);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


    @Override
    public void runOpMode() {
        clawLeft = hardwareMap.servo.get("vl");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        clawRight = hardwareMap.servo.get("vr");


        armMotor = hardwareMap.dcMotor.get("arm");
        slideMotor = hardwareMap.dcMotor.get("slide");
        wristServo = hardwareMap.servo.get("wr");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = new Claw(clawLeft,clawRight);
        slide = new Slide(slideMotor);
        arm = new Arm(armMotor);
        Wrist wrist = new Wrist(wristServo);
        wristServo.setDirection(Servo.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        initGyro();
        arm.down();
        sleep(100);
        sleep(100);
        claw.open();
        sleep(100);
        wrist.flat();
        waitForStart();

        //Auto Code
        strafeLeft(8.4,0.5); //TODO: The Value
        arm.down();
        sleep(100); //TODO: The Value
        forward(13.5,0.6);
        slide.expandDown();
        sleep(500);
        claw.close();
        sleep(500);
        slide.contract();
        sleep(550);
        turnLeft(-130,0.6); //TODO: The Value

        sleep(500);
        forward(6.5,0.5);
        sleep(500);
        arm.up();
        sleep(700);
        slide.expandUP();
        sleep(1500); //TODO: The Value
        claw.open();
        sleep(700);
        armMotor.setTargetPosition(armMotor.getCurrentPosition()+30);
        sleep(250);
        back(6,0.4);
        sleep(1000);
        slide.contract();
        sleep(700);
        arm.down();
        turnRight(-130,0.6); //TODO: The Value
        sleep(500);
        back(20,1);//TODO: The Value
        //Auto Code End












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
            telemetry.addData("Is ARM Busy?", armMotor.isBusy());
            telemetry.addData("Is Slide Busy?", slideMotor.isBusy());
            telemetry.addData("Is Front Left Busy?", frontLeft.isBusy());
            telemetry.addData("Is Front Right Busy?", frontRight.isBusy());
            telemetry.addData("Is Back Left Busy?", backLeft.isBusy());
            telemetry.addData("Is Back Right Busy?", backRight.isBusy());
            telemetry.addData("ARM POWER:", armMotor.getPower());
            telemetry.addData("SLIDE POWER:", slideMotor.getPower());

            telemetry.update();

        }
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
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
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
