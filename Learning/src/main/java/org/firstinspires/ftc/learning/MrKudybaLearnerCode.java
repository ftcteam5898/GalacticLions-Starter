package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mr.Kudyba's TeleOp", group="Starter Code")
public class MrKudybaLearnerCode extends LinearOpMode {

    //runOpMode starts once we press init
    @Override
    public  void  runOpMode() {
        //Any code that we write BEFORE waitForStart() will run when we
        //press init but before we press play.
        DcMotor motorLeft = hardwareMap.dcMotor.get("left");
        DcMotor motorRight = hardwareMap.dcMotor.get("right");

        // We need to flip the left motor so it goes the same way as the right motor.
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //This lets the code wait for us to press the play button
        waitForStart();

        //If we press stop BEFORE pressing play, stop the code.
        if(isStopRequested()) return;

        //while (I've hit play())
        while (opModeIsActive()) {
            //Do all code here as long as we have pressed play and
            // not pressed stop.
            motorRight.setPower(gamepad1.right_trigger);
            motorLeft.setPower(gamepad1.left_trigger);

            //IF we press the right bumper do the thing in the { }
            if (gamepad1.right_bumper){
                motorRight.setPower(-1);
            }
            //If we let go of the right bumper do the else
            else {
                motorRight.setPower(0);
            }

            //IF we press the left bumper do the thing in the { }
            if (gamepad1.left_bumper){
                motorLeft.setPower(-1);
            }
            //If we let go of the left bumper do the else
            else {
                motorLeft.setPower(0);
            }
        }
    }
}
