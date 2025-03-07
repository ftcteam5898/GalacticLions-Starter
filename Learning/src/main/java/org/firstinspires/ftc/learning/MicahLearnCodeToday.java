package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Micah's First Code", group="Starter Code")
public class MicahLearnCodeToday extends LinearOpMode {

    //runOpMode starts once we press init
    @Override
    public void runOpMode(){
        //Any code the we write BEFORE waitforstart() will run when we
        //press init but before we press play

        DcMotor Pikachu = hardwareMap.dcMotor.get("pokemonGo");
        DcMotor Raichu = hardwareMap.dcMotor.get("pokemonTCGP");

        Pikachu.setPower(1);
        
        //this lets the code wait for us to press the play button
        waitForStart();

        //if we press stop before pressing play, stop the code
        if (isStopRequested()) return;

        //while (I've hit play())
        while (opModeIsActive()) {
            //do all code here as long as we have pressed play and
            //not presses stop

            Raichu.setPower(1);
        }
    }
}
