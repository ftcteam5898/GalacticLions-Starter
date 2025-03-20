package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Emma's TeleOp", group="Starter Code")
public class EmmasTeleOp extends LinearOpMode {

    // pressing init starts runOpMode
    @Override
    public void runOpMode() {

        //any code we write before waitForStart() will run when we
        //press init but before we press play
        DcMotor motorLeft = hardwareMap.dcMotor.get("left");
        DcMotor motorRight = hardwareMap.dcMotor.get("right");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo arm = hardwareMap.get(Servo.class, "arm");
        CRServo spin = hardwareMap.crservo.get("spin");

        DistanceSensor dist = hardwareMap.get(DistanceSensor.class, "Daniela");

        //Create our touch sensor
        TouchSensor sensy = hardwareMap.get(TouchSensor.class, "sensy");
        boolean flip = false; //we use this to flip the direction of the CR servo
        double power = .5; //This powers our CR servo

        ColorSensor color = hardwareMap.get(ColorSensor.class, "Color");

        //color sensor stuffs
        int red = color.red();
        int blue = color.blue();
        int green = color.green();
        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.addData("Green", green);
        telemetry.update();
        if (gamepad1.a) {
            color.enableLed(true);
        } else if (gamepad1.b) {
            color.enableLed(false);
        }


        //Getting our servo into position before we hit start
        arm.setPosition(.5);

        // While init wait for the play button
        waitForStart();

        // All the code to make the robot go once we have pressed
        // play

        // If stop is requested STAWP
        if (isStopRequested()) return;

        // while (I've hit play())
        while (opModeIsActive()) {
            //Do all code here as long as we have pressed play and
            // not pressed stop

            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            //Distance sensor stuff
            telemetry.addData("range", String.format("%.01f cm", dist.getDistance(DistanceUnit.CM)));

            //Debugging code that's no longer necessary
            //telemetry.addData("Button pressed:", sensy.isPressed());


            if (sensy.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
                flip = true;
                // else telemetry.addData("Touch Sensor", "Not Pressed");
            }
            else {
                telemetry.addData("Touch Sensor", "Not Pressed");
                //spin.setPower(-1);
                flip = false;
            };

            if (flip) {
                power = power * -1;
            }

            spin.setPower(power);

            telemetry.update();

            //Controlling our arm servo with the thumb sticks
            //arm.setPosition(gamepad1.left_stick_x);

            //Controlling our servo with the dpad
            if (gamepad1.dpad_left){
                arm.setPosition(0);
            }

            else if (gamepad1.dpad_right){
                arm.setPosition(1);
            }
            else arm.setPosition(.5);



        }
    }
}