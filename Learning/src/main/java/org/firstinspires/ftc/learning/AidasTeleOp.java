package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="AidasTeleOp", group="Starter Code")
public class AidasTeleOp extends LinearOpMode {
    //runOpMode starts once we press init
    @Override
    public void runOpMode(){
        //Any code that we write BEFORE waitForStart() will run when
        //press init but before we press play.
        DcMotor motorLeft = hardwareMap.dcMotor.get("lob");
        DcMotor motorRight = hardwareMap.dcMotor.get("rob");


        //This lets the code wait for us to press the play button
        waitForStart();

        //If we press stop BEFORE pressing play, stop the code.
        if (isStopRequested()) return;

        // while (I've hit play())
        while (opModeIsActive()) {
           //Do all code here as long as we have pressed play and
           // not pressed stop.
            motorRight.setPower(gamepad1.right_trigger);
            motorLeft.setPower(gamepad1.left_trigger);

        }
    }
}
