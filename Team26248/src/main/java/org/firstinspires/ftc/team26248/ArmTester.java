package org.firstinspires.ftc.team26248;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
                armMotor.setPower(0.5);

            }
            else if(gamepad1.dpad_down) {
                armMotor.setPower(-0.5);

            }
            else {
                armMotor.setPower(0);

            }
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
