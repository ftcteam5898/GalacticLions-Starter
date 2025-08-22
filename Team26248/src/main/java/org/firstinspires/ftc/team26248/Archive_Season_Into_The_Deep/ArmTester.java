package org.firstinspires.ftc.team26248.Archive_Season_Into_The_Deep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Disabled
@TeleOp(name="ArmTester", group="Test")
public class ArmTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        boolean direction = true;
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        int armPosition = 0;

        while(opModeIsActive()) {

            if(direction) {
                armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else {
                armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(gamepad1.a) {
                direction = !direction;
            }
            if(gamepad1.dpad_up) {
                armPosition += 10;
            }
            else if(gamepad1.dpad_down) {
                armPosition -= 10;
            }
            else if(gamepad1.left_trigger > 0.5) {
                armPosition -= 100;
            }
            else if(gamepad1.right_trigger > 0.5) {
                armPosition += 100;
            }
            else if(gamepad1.left_bumper) {
                armPosition -= 1;
            }
            else if(gamepad1.right_bumper) {
                armPosition += 1;
            }

            armMotor.setTargetPosition(armPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            armMotor.setPower(0.5);

            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
