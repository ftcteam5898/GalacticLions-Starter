package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="FortNiteShawty", group="Starter Code")
public class EricsTeleOp extends LinearOpMode {
    //runOpMode starts once we press init
    @Override
    public void runOpMode() {
        //Any code that we write BEFORE waitForStart() will run when we
        // press init but before we press play.
        DcMotor Peely = hardwareMap.dcMotor.get("DurrBurger");
        DcMotor BigDill = hardwareMap.dcMotor.get("Brat");

        Servo EliXiao = hardwareMap.get(Servo.class, "EliXiao");
        CRServo FletcherKane = hardwareMap.get("CampRaineyMountian");

        //We need to flip the left motor so it goes the same way as the right
        Peely.setDirection(DcMotorSimple.Direction.REVERSE);

        //Getting our servo into position BEFORE we hit start
        EliXiao.setPosition(.5);

        //This lets the code wait for us to press the play button
        waitForStart();

        //If we press stop BEFORE pressing play, stop the code.
        if(isStopRequested()) return;

        //while (I've hit play())
        while (opModeIsActive()) {
            Peely.setPower(-gamepad1.left_stick_y);
            BigDill.setPower(-gamepad1.right_stick_y);

            //Controlling our arm servo with the thumb stick
//            EliXiao.setPosition(gamepad1.left_stick_x);

            //Controlling our arm servo with the dpad
            if (gamepad1.dpad_left){
                EliXiao.setPosition(0);
            }
            else if (gamepad1.dpad_right){
                EliXiao.setPosition(1);
            }
            else EliXiao.setPosition(.5);

            //Controlling our FletcherKane servo with the triggers
            FletcherKane.setPower(gamepad1.right_trigger);
            FletcherKane.setPower(-gamepad1.left_trigger);

               // Trigger & Bumper driving code.
//            //Do all code here as long as we have pressed play and
//            //not pressed stop.
//            BigDill.setPower(gamepad1.right_trigger);
//            Peely.setPower(gamepad1.left_trigger);
//
//            //If we press the right bumper do the thing in th { }
//            if(gamepad1.left_bumper){
//                Peely.setPower(-1);
//            }
//            //If we let go of the right bumper do the else
//            else {
//                Peely.setPower(0);
//            }
//
//            //If we press the right bumper do the thing in the { }
//            if(gamepad1.right_bumper){
//                BigDill.setPower(-1);
//            }
//            //If we let go of the right bumper do the else
//            else {
//                BigDill.setPower(0);
//            }
        }
    }

}