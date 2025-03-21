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

@TeleOp(name="Micah's First Code", group="Starter Code")
public class MicahLearnCodeToday extends LinearOpMode {

    //runOpMode starts once we press init
    @Override
    public void runOpMode(){
        //Any code the we write BEFORE waitforstart() will run when we
        //press init but before we press play

        //adding code for the motors
        DcMotor Pikachu = hardwareMap.dcMotor.get("Pikachu");
        DcMotor Raichu = hardwareMap.dcMotor.get("Raichu");

        //adding code for the servos
        Servo Gigalith = hardwareMap.get(Servo.class,"Gigalith");
        CRServo Boldore = hardwareMap.crservo.get("Boldore");

        //adding code for the sensors
        DistanceSensor Tyrunt = hardwareMap.get(DistanceSensor.class,"Tyrunt");
        ColorSensor Grookey = hardwareMap.get(ColorSensor.class,"Grookey");
        TouchSensor Budew = hardwareMap.get(TouchSensor.class,"Budew");
        boolean Roserade = false; //we use this to flip the direction of the CRServo
        double Tyrogue = .5; //this powers our CRServo

        //we need to flip the left motor so it goes the same way as the right motor
        Pikachu.setDirection(DcMotorSimple.Direction.REVERSE);

        //getting our servo into position before we press start
        Gigalith.setPosition(0.5);

        //this lets the code wait for us to press the play button
        waitForStart();

        //if we press stop before pressing play, stop the code
        if (isStopRequested()) return;
        //while (I've hit play())
        while (opModeIsActive()) {
            Pikachu.setPower(-gamepad1.left_stick_y);
            Raichu.setPower(-gamepad1.right_stick_y);

            //distance sensor stuff
            telemetry.addData("deviceName", Tyrunt.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", Tyrunt.getDistance(DistanceUnit.MM)));

            //color sensor stuff
            int red = Grookey.red();
            int blue = Grookey.blue();
            int green = Grookey.green();
            telemetry.addData("red: ", red);
            telemetry.addData("blue: ", blue);
            telemetry.addData("green: ",green);
            //debugging code that's no longer necessary
            //telemetry.addData("Budew pressed:", Budew.isPressed());

            if (Budew.isPressed()){
                telemetry.addData("Budew", "Is Pressed");
                Roserade = true;
                //Boldore.setPower(1);
            }
            else {
                telemetry.addData("Budew", "Not Pressed");
                Roserade = false;
                //Boldore.setPower(-1);
            }

            if (Roserade) {
                Tyrogue = Tyrogue * -1;
            }
            Boldore.setPower(Tyrogue);

            telemetry.update();

          /*  if (gamepad1.a){
                Grookey.enableLed(true);
            }
            else if (gamepad1.b) {
                Grookey.enableLed(false);
            }
            if (red>2000){
                Boldore.setPower(1);
            }
            else {
                Boldore.setPower(0);
            }*/

            //

            //controlling the servo with the x axis joystick
            //Gigalith.setPosition(gamepad1.left_stick_x);

            //controlling the servo with the dpad
            if (gamepad1.dpad_left){
                Gigalith.setPosition(0);
            }
            else if (gamepad1.dpad_right){
                Gigalith.setPosition(1);
            }
            else Gigalith.setPosition(0.5);

            //controlling the CR servo with the triggers
//            Boldore.setPower(gamepad1.right_trigger);
//            Boldore.setPower(-gamepad1.left_trigger);

            //Trigger & Bumber driving code
//            //do all code here as long as we have pressed play and
//            //not pressed stop
//            //Pikachu.setPower(0); //only way to stop a motor
//            Raichu.setPower(gamepad1.right_trigger);
//            Pikachu.setPower(gamepad1.left_trigger);
//            //IF we press the right bumber do the thing in the { }
//            if (gamepad1.right_bumper){
//                Raichu.setPower(-1);
//            }
//            //IF we let go of right bumber, do something else
//            else {
//                Raichu.setPower(0);
//            }
//            if (gamepad1.left_bumper){
//                Pikachu.setPower(-1);
//            }
//            else {
//                Pikachu.setPower(0);
//           }
        }
    }
}