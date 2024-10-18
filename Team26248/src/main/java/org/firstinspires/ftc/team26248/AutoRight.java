package org.firstinspires.ftc.team26248;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Auto_Right_Park", group="Starter Code")
public class AutoRight extends LinearOpMode {
    DcMotor motorFrontLeft = hardwareMap.dcMotor.get("fl");
    DcMotor motorFrontRight = hardwareMap.dcMotor.get("fr");
    DcMotor motorBackLeft = hardwareMap.dcMotor.get("bl");
    DcMotor motorBackRight = hardwareMap.dcMotor.get("br");
    Servo clawLeftMotor = hardwareMap.servo.get("vl");
    Servo clawRightMotor = hardwareMap.servo.get("vr");


    double cpr = 1425.1;
    double gearratio = 1;

    @Override
    public void runOpMode() {
        //Set the Direction of the Motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        clawRightMotor.setDirection(Servo.Direction.REVERSE);
        clawLeftMotor.setDirection(Servo.Direction.FORWARD);
        waitForStart();

        if (isStopRequested()) return;

        //Run the OpMode
        while (opModeIsActive()) {


        }
    }
}