package org.firstinspires.ftc.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mr.Kudyba's TeleOp", group="Starter Code")
public class MrKudybaLearnerCode extends LinearOpMode {

    @Override
    // This method starts running once we hit init.
    public  void  runOpMode() {
        //If we want anything to happen before we hit play
        DcMotor leftMotor = hardwareMap.dcMotor.get("left");
        DcMotor rightMotor = hardwareMap.dcMotor.get("right");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Wait for the driver to press play
        waitForStart();

        //Check to see if the driver pressed stop, if so STOP.
        if (isStopRequested()){
            return;
        }

        while (opModeIsActive()){
            //This is all the code we want to do when we press play
            if (gamepad1.left_bumper){
                leftMotor.setPower(1);
            }
            else leftMotor.setPower(0);

            if (gamepad1.right_bumper){
                rightMotor.setPower(1);
            }
            else rightMotor.setPower(0);
        }

    }
}
