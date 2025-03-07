package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Micah's First Code", group="Starter Code")
public class MicahLearnCodeToday extends LinearOpMode {

    //runOpMode starts once we press init
    @Override
    public void runOpMode(){
        //Any code the we write BEFORE waitforstart() will run when we
        //press init but before we press play
        DcMotor Pikachu = hardwareMap.dcMotor.get("pokemonGo");
        DcMotor Raichu = hardwareMap.dcMotor.get("pokemonTCGP");
        Pikachu.setDirection(DcMotorSimple.Direction.REVERSE);
        //this lets the code wait for us to press the play button
        waitForStart();
        //if we press stop before pressing play, stop the code
        if (isStopRequested()) return;
        //while (I've hit play())
        while (opModeIsActive()) {
            //do all code here as long as we have pressed play and
            //not pressed stop
            //Pikachu.setPower(0); //only way to stop a motor
            Raichu.setPower(gamepad1.right_trigger);
            Pikachu.setPower(gamepad1.left_trigger);
            //IF we press the right bumber do the thing in the { }
            if (gamepad1.right_bumper){
                Raichu.setPower(-1);
            }
            //IF we let go of right bumber, do something else
            else {
                Raichu.setPower(0);
            }
            if (gamepad1.left_bumper){
                Pikachu.setPower(-1);
            }
            else {
                Pikachu.setPower(0);
            }
        }
    }
}
