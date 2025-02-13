package org.firstinspires.ftc.team26248;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// TODO: Maxium for high basket is -2000
@TeleOp(name="SlidePositionTest", group="Test")
public class slidetest extends LinearOpMode {
    @Override

    public void runOpMode() {
        boolean direction = true;
        DcMotor slideMotor = hardwareMap.dcMotor.get("slide");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition());
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                direction = !direction;
                if(direction) {
                    slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                else {
                    slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                }
            }
            slideMotor.setPower(.5);
            if(gamepad1.dpad_up) {
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + 20);
            }
            else if(gamepad1.dpad_down) {
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - 20);
            }
            telemetry.addData("Slide Current Position", slideMotor.getCurrentPosition());
            telemetry.addData("Slide Target Position", slideMotor.getTargetPosition());
            telemetry.update();
        }
    }

}
