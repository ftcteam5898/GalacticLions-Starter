package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Mr.Kudyba's TeleOp", group="Starter Code")
public class MrKudybaLearnerCode extends LinearOpMode {

    //runOpMode starts once we press init
    @Override
    public  void  runOpMode() {
        //Any code that we write BEFORE waitForStart() will run when we
        //press init but before we press play.
        DcMotor motorLeft = hardwareMap.dcMotor.get("left");
        DcMotor motorRight = hardwareMap.dcMotor.get("right");

        Servo arm = hardwareMap.get(Servo.class, "arm");

        CRServo spin = hardwareMap.get(CRServo.class, "spin");

        DistanceSensor dist = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        //Create our touch sensor
        TouchSensor button = hardwareMap.get(TouchSensor.class, "button");
        boolean flip = false; //We use this to flip the direction of the CR servo
        double power = .5; //This powers our CR servo

        // We need to flip the left motor so it goes the same way as the right motor.
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //This lets the code wait for us to press the play button
        waitForStart();

        //If we press stop BEFORE pressing play, stop the code.
        if(isStopRequested()) return;

        //while (I've hit play())
        while (opModeIsActive()) {
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            // DistanceSensor stuff
            telemetry.addData("deviceName", dist.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", dist.getDistance(DistanceUnit.CM)));

            //Debugging code that's no longer necessary
            //telemetry.addData("Button pressed:", button.isPressed());

            if (button.isPressed()){
                telemetry.addData("Touch Sensor", "Is Pressed");
                flip = true;
                //spin.setPower(1);
            }
            else {
                telemetry.addData("Touch Sensor", "Not Pressed");
                //spin.setPower(-1);
                flip = false;
            }

            if (flip) {
                power = power * -1;
            }

            spin.setPower(power);

            telemetry.update();


            //Trigger & Bumper driving code.
//            //Do all code here as long as we have pressed play and
//            // not pressed stop.
//            motorRight.setPower(gamepad1.right_trigger);
//            motorLeft.setPower(gamepad1.left_trigger);
//
//            //IF we press the right bumper do the thing in the { }
//            if (gamepad1.right_bumper){
//                motorRight.setPower(-1);
//            }
//            //If we let go of the right bumper do the else
//            else {
//                motorRight.setPower(0);
//            }
//
//            //IF we press the left bumper do the thing in the { }
//            if (gamepad1.left_bumper){
//                motorLeft.setPower(-1);
//            }
//            //If we let go of the left bumper do the else
//            else {
//                motorLeft.setPower(0);
//            }
        }
    }
}
