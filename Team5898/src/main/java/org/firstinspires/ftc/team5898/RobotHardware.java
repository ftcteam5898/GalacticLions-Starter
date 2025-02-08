package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotHardware {
    // Declare Motors
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public DcMotorEx slideLeft, slideRight;

    // Declare Servos
    public Servo claw, grabber, leftOuttake, rightOuttake, wrist, leftIntake, rightIntake;

    // Sensors
    public IMU imu;

    // Hardware Map Reference
    private HardwareMap hardwareMap;

    // Constructor
    public RobotHardware(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

    // Initialize hardware
    public void init() {
        // Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        slideLeft = hardwareMap.get(DcMotorEx.class, "SL");
        slideRight = hardwareMap.get(DcMotorEx.class, "SR");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake when power is zero
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to run without encoders initially
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos
        claw = hardwareMap.get(Servo.class, "Claw");
        grabber = hardwareMap.get(Servo.class, "Grabber");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        leftOuttake = hardwareMap.get(Servo.class, "LeftOuttake");
        rightOuttake = hardwareMap.get(Servo.class, "RightOuttake");
        leftIntake = hardwareMap.get(Servo.class, "LeftIntake");
        rightIntake = hardwareMap.get(Servo.class, "RightIntake");

        // Sensors
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }



    // Helper method to set all motors to a specific run mode
    private void setMotorRunMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
        slideLeft.setMode(mode);
        slideRight.setMode(mode);
    }

}