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

@Disabled
@Autonomous(name = "",group = "Autonomous")
public class Autonomous_Template extends LinearOpMode {
    DcMotor frontLeft,frontRight, backLeft, backRight, armMotor, slideMotor;
    Servo clawLeft, clawRight;


    //DO NOT CHANGE
    double cpr = 537.7; //312 RPM Gobilda
    double gearRatio = 1;
    double diameter = 4.094; //for Gobilda 104mm Mecanum wheel
    double cpi = (cpr*gearRatio) / (Math.PI * diameter);
    double bias = 0.94;
    double strafeBias = 1.0;
    double conversion = cpi * bias;
    Claw claw;
    Slide slide;
    Arm arm;
    IMU imu;

    //Modules for Claw, Slide, and Arm
    public class Claw {
        private Servo clawLeft = hardwareMap.servo.get("vl");
        private Servo clawRight = hardwareMap.servo.get("vr");

        //Change Claw Status Here
        private final double CLAW_LEFT_OPEN = 0.25;
        private final double CLAW_RIGHT_OPEN = 0.5;
        private final double CLAW_LEFT_CLOSE = 0.5;
        private final double CLAW_RIGHT_CLOSE = 0.75;
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
        private DcMotor slideMotor = hardwareMap.dcMotor.get("slide");

        //Change Slide Status Here
        private final int slide_expanded = 0; //TODO:need to change
        private final int slide_contracted = 0;//TODO:need to change

        public Slide(DcMotor slideMotor){
            this.slideMotor = slideMotor;
        }
        public void expand(){
            slideMotor.setTargetPosition(slide_expanded);
        }
        public void contract(){
            slideMotor.setTargetPosition(slide_contracted);
        }
    }

    public class Arm{
        private DcMotor armMotor = hardwareMap.dcMotor.get("arm");

        //Change Arm Status Here
        private final int arm_up = 0; //TODO:need to change
        private final int arm_down = 0;//TODO:need to change

        public Arm(DcMotor armMotor){
            this.armMotor = armMotor;
        }
        public void up(){
            armMotor.setTargetPosition(arm_up);
        }
        public void down(){
            armMotor.setTargetPosition(arm_down);
        }
    }


    @Override
    public void runOpMode() {

        //Initialization for motors
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        clawRight = hardwareMap.servo.get("vr");
        clawLeft = hardwareMap.servo.get("vl");
        armMotor = hardwareMap.dcMotor.get("arm");
        slideMotor = hardwareMap.dcMotor.get("slide");

        //Initialization for Encoders
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialization for Modules
        claw = new Claw(clawLeft,clawRight);
        slide = new Slide(slideMotor);
        arm = new Arm(armMotor);

        arm.down();

        //Optional:claw.open();

        initGyro();

        waitForStart();


        //Code Here
        //Example: forward(12,0.5);
        //Turn Example: turnWithGyro(90,0.5); Clock wise 90 degrees
        //Strafe Example: strafeRight(12,0.5);




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


    public void moveToPosition(double inches, double speed) {
        int move = (int) (Math.round(inches * conversion));
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

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
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
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Working...", " ");
            telemetry.update();
        }
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
