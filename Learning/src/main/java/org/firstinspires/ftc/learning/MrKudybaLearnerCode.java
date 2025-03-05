package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mr.Kudyba's TeleOp", group="Starter Code")
public class MrKudybaLearnerCode extends LinearOpMode {

    @Override
    // This method starts running once we hit init.
    public  void  runOpMode() {
        //If we want anything to happen before we hit play
        DcMotor rightMotor = hardwareMap.dcMotor.get("left");
        DcMotor leftMotor = hardwareMap.dcMotor.get("right");

        //Wait for the driver to press play
        waitForStart();

        //Check to see if the driver pressed stop, if so STOP.
        if (isStopRequested()){
            return;
        }

        while (opModeIsActive()){
            //This is all the code we want to do when we press play

        }

    }
}
