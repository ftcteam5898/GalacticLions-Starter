package org.firstinspires.ftc.learning;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    // Declare Motors
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;

    // Declare Servos
    //public Servo claw;

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

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Servos
        //claw = hardwareMap.get(Servo.class, "Claw");

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
    }
}