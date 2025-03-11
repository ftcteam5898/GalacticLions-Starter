package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="FortNiteShawty", group="Starter Code")
public class EricsTeleOp extends LinearOpMode {
    //runOpMode starts once we press init
    @Override
    public void runOpMode() {
        //Any code that we write BEFORE waitForStart() will run when we
        // press init but before we press play.
        DcMotor Peely = hardwareMap.dcMotor.get("DurrBurger");
        DcMotor BigDill = hardwareMap.dcMotor.get("Brat");

        //We need to flip the left motor so it goes the same way as the right
        Peely.setDirection(DcMotorSimple.Direction.REVERSE);


        //This lets the code wait for us to press the play button
        waitForStart();

        //If we press stop BEFORE pressing play, stop the code.
        if(isStopRequested()) return;

        //while (I've hit play())
        while (opModeIsActive()) {
            //Do all code here as long as we have pressed play and
            //not pressed stop.
            BigDill.setPower(gamepad1.right_trigger);
            Peely.setPower(gamepad1.left_trigger);

            //If we press the right bumper do the thing in th { }
            if(gamepad1.left_bumper){
                Peely.setPower(-1);
            }
            //If we let go of the right bumper do the else
            else {
                Peely.setPower(0);
            }

            //If we press the right bumper do the thing in the { }
            if(gamepad1.right_bumper){
                BigDill.setPower(-1);
            }
            //If we let go of the right bumper do the else
            else {
                BigDill.setPower(0);
            }
        }
    }

}