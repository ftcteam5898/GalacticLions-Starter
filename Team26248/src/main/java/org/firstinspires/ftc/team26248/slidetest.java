package org.firstinspires.ftc.team26248;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
// TODO: Maxium for high basket is -2000
@TeleOp(name="SlidePositionTest", group="Test")
public class slidetest extends LinearOpMode {
    @Override

    public void runOpMode() {
        boolean direction = true;
        DcMotor slideMotor = hardwareMap.dcMotor.get("slide");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            if(gamepad1.dpad_up) {
                slideMotor.setPower(0.5);
                telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
                telemetry.update();
            }
            else if(gamepad1.dpad_down) {
                slideMotor.setPower(-0.5);
                telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
                telemetry.update();
            }
            else {
                slideMotor.setPower(0);
                telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }

}
