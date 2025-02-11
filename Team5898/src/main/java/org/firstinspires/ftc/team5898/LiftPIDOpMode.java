package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Lift PID Control", group="Competition")
public class LiftPIDOpMode extends LinearOpMode {
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                robot.syncLift(0.8); // Raise lift with PID correction
            } else if (gamepad1.dpad_down) {
                robot.syncLift(-0.5); // Lower lift with PID correction
            } else {
                robot.syncLift(0); // Stop lift
            }
        }
    }
}
