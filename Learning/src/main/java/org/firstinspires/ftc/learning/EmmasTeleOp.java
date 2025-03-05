package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Emma's TeleOp", group="Starter Code")
public class EmmasTeleOp extends LinearOpMode {

    // pressing init starts runOpMode
    @Override
    public void runOpMode() {

        // While init wait for the play button
        waitForStart();

        // If stop is requested STAWP
        if (isStopRequested()) return;

        // All the code to make the robot go once we have pressed
        // play
        while (opModeIsActive()) {

        }
    }
}