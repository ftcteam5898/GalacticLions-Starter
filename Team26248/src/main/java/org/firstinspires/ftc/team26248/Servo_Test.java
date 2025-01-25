package org.firstinspires.ftc.team26248;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Linear_Servo_Test", group="Test")
public class Servo_Test extends LinearOpMode {

    private Servo servoRight; // 右侧伺服
    private Servo servoLeft;  // 左侧伺服

    // 初始化伺服的默认位置
    private double servoRightPosition; // 中位
    private double servoLeftPosition;  // 中位

    @Override
    public void runOpMode() {
        // 获取硬件映射
        servoRight = hardwareMap.servo.get("vr");
        servoLeft = hardwareMap.servo.get("vl");

        // 将伺服移动到初始位置
        servoRight.setPosition(servoRightPosition);
        servoLeft.setPosition(servoLeftPosition);

        // 显示初始化完成
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo Right Position", servoRightPosition);
        telemetry.addData("Servo Left Position", servoLeftPosition);
        telemetry.update();

        // 等待用户按下 PLAY 按钮
        waitForStart();

        while (opModeIsActive()) {
            // 检测按键并调整伺服位置
            if (gamepad1.a) {
                servoRightPosition += 0.1; // 右伺服增加
            } else if (gamepad1.b) {
                servoRightPosition -= 0.1; // 右伺服减少
            }

            if (gamepad1.y) {
                servoLeftPosition += 0.1; // 左伺服增加
            } else if (gamepad1.x) {
                servoLeftPosition -= 0.1; // 左伺服减少
            }

            // 限制伺服位置范围在 [0, 1]
            servoRightPosition = Math.max(0, Math.min(1, servoRightPosition));
            servoLeftPosition = Math.max(0, Math.min(1, servoLeftPosition));

            // 更新伺服位置
            servoRight.setPosition(servoRightPosition);
            servoLeft.setPosition(servoLeftPosition);

            // 输出伺服位置到 telemetry
            telemetry.addData("Servo Right Position", servoRightPosition);
            telemetry.addData("Servo Left Position", servoLeftPosition);
            telemetry.update();

            // 短暂延迟以避免过于频繁地检测输入
            sleep(100);
        }
    }
}
