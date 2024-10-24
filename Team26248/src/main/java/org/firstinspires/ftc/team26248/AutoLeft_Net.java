package org.firstinspires.ftc.team26248;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
// 导入Range类
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AutoLeft_Net", group = "Autonomous")

public class AutoLeft_Net extends LinearOpMode {

    // 定义PIDController类
//    class PIDController {
//        private double kP;
//        private double kI;
//        private double kD;
//        private double integralSum;
//        private double lastError;
//        private double output;
//
//        public PIDController(double kP, double kI, double kD){
//            this.kP = kP;
//            this.kI = kI;
//            this.kD = kD;
//            integralSum = 0;
//            lastError = 0;
//        }
//
//        public double calculate(double target, double current){
//            double error = target - current;
//            integralSum += error;
//            double derivative = error - lastError;
//            lastError = error;
//
//            output = (kP * error) + (kI * integralSum) + (kD * derivative);
//            return output;
//        }
//
//        public void reset(){
//            integralSum = 0;
//            lastError = 0;
//        }
//    }

    DcMotor frontleft, frontright, backleft, backright, armMotor;
    Servo clawLeft, clawRight;

    // motor counts per rotation (ticks/pulses per rotation)
    // check motor specs from manufacturer
    double cpr = 1425.1;

    // adjust gearRatio if you have geared up or down your motors
    double gearRatio = 1;

    // wheel diameter in inches
    double diameter = 3.779;

    // counts per inch: cpr * gear ratio / (pi * diameter (in inches))
    double cpi = (cpr * gearRatio) / (Math.PI * diameter);

    // use calibrate auto to check this number before proceeding
    double bias = 0.94; // adjust based on calibration opMode

    double strafeBias = 0.9; // change to adjust only strafing movement

    double conversion = cpi / bias;

    IMU imu;

    @Override
    public void runOpMode() {

        initGyro();

        // setup motors
        frontleft = hardwareMap.dcMotor.get("fl");
        frontright = hardwareMap.dcMotor.get("fr");
        backleft = hardwareMap.dcMotor.get("bl");
        backright = hardwareMap.dcMotor.get("br");
        clawLeft = hardwareMap.servo.get("vl");
        clawRight = hardwareMap.servo.get("vr");
        armMotor = hardwareMap.dcMotor.get("arm");

        // reverse the left side motors
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        // 初始化机械臂电机
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始化PID控制器
//        double kP = 0.01; // 需要根据实际情况调整
//        double kI = 0.0;
//        double kD = 0.0;
//        PIDController pid = new PIDController(kP, kI, kD);

        // 设置机械臂的目标位置
//        int targetAngle = 45; // 目标角45度
//        int maxAngle = 180; // 机械臂的最大角度
//        int countsPerRev = 1120; // 电机每转的编码器计数，需要根据实际电机调整
//        double armGearRatio = 1.0; // 机械臂的齿轮比
//
//        int targetPosition = (int)((targetAngle / (double)maxAngle) * (countsPerRev * armGearRatio));
//
//        // wait for Start to be pressed
//        waitForStart();

        // 逆时针旋转90度
        turnLeft(90, 0.5);

        // 前进22英寸
        forward(22, 0.7);

        // 松开舵机的爪子
        clawLeft.setPosition(0);
        clawRight.setPosition(1);

        // 后退120英寸
        back(120, 0.8);
//
//        // 提升机械臂到45度
//        while (opModeIsActive()) {
//            int currentPosition = armMotor.getCurrentPosition();
//            double power = pid.calculate(targetPosition, currentPosition);
//
//            // 抛气量金让机械臂速度格局（可选）
//            double gravityCompensation = 0.1; // 根据实际情况调整
//            power += gravityCompensation;
//
//            // 限制力量在-1到1之间
//            power = Range.clip(power, -1.0, 1.0);
//            armMotor.setPower(power);
//
//            // 判断是否达到目标位置
//            if (Math.abs(currentPosition - targetPosition) < 10) { // 允许的误差范围
//                break;
//            }
//
//            // Telemetry输出（可选）
//            telemetry.addData("Target Position", targetPosition);
//            telemetry.addData("Current Position", currentPosition);
//            telemetry.addData("Motor Power", power);
//            telemetry.update();
//
//            // 添加短暫的延迟，防止循环过快
//            sleep(20);
//        }
//
//        // 停止机械臂电机
//        armMotor.setPower(0);

        // 接下来是您的自动程序，例如移动机器人
        // 例如，调用strafeRight(48, 0.9);
        strafeRight(48, 0.9);
    }

    /**
     * 用于使机器人前进指定英寸。
     *
     * @param inches 要前进的距离，单位为英寸
     * @param speed  速度范围为[0,1]
     */
    public void forward(double inches, double speed) {
        moveToPosition(inches, speed);
    }

    /**
     * 用于使机器人后退指定英寸。
     *
     * @param inches 要后退的距离，单位为英寸
     * @param speed  速度范围为[0,1]
     */
    public void back(double inches, double speed) {
        moveToPosition(-inches, speed);
    }

    /**
     * 使机器人向左旋转。
     *
     * @param degrees 要旋转的角度
     * @param speed   速度范围为[0,1]
     */
    public void turnLeft(double degrees, double speed) {
        turnWithGyro(degrees, -speed);
    }

    /**
     * 使机器人向右旋转。
     *
     * @param degrees 要旋转的角度
     * @param speed   速度范围为[0,1]
     */
    public void turnRight(double degrees, double speed) {
        turnWithGyro(degrees, speed);
    }

    /**
     * 横移到左侧。
     *
     * @param inches 要横移的距离，单位为英寸
     * @param speed  速度范围为[0,1]
     */
    public void strafeLeft(double inches, double speed) {
        strafeToPosition(-inches, speed);
    }

    /**
     * 横移到右侧。
     *
     * @param inches 要横移的距离，单位为英寸
     * @param speed  速度范围为[0,1]
     */
    public void strafeRight(double inches, double speed) {
        strafeToPosition(inches, speed);
    }

    /*
     * 这个函数的目的是简单地驱动前进或后退。
     * 要向后驾驶，只需将英寸输入设为负数。
     */
    public void moveToPosition(double inches, double speed) {
        int move = (int) (Math.round(inches * conversion));
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);

        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            telemetry.addData("Busy...", "");
            telemetry.update();
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

    /*
     * 这个函数使用Hub IMU集成陀螺仪精确旋转一定角度（+/- 5）。
     * 角度应始终为正，若要左转则使speedDirection为负。
     */
    public void turnWithGyro(double degrees, double speedDirection) {
        // 创建一个对象来接收IMU角度
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // 初始化

        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // 是否应为负数？
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();

        double first;
        double second;

        // 右转
        if (speedDirection > 0) {
            if (degrees > 10) {
                first = (degrees - 10) + devertify(yaw);
            } else {
                first = devertify(yaw);
            }
            second = degrees + devertify(yaw);
        }

        // 左转
        else {
            if (degrees > 10) {
                first = devertify(-(degrees - 10) + devertify(yaw));
            } else {
                first = devertify(yaw);
            }
            second = devertify(-degrees + devertify(yaw));
        }

        // 到达位置
        double firsta = convertify(first - 5);
        double firstb = convertify(first + 5);
        turnWithEncoder(speedDirection);

        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {// 是否在范围内？
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // 是否应为负数？
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        } else {
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {// 是否在范围内？
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // 是否应为负数？
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }

        double seconda = convertify(second - 5);// 175
        double secondb = convertify(second + 5);// -175
        turnWithEncoder(speedDirection / 3);

        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {// 是否在范围内？
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // 是否应为负数？
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {// 是否在范围内？
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // 是否应为负数？
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
     * 这个函数使用编码器横移（向左或向右）。
     * 英寸输入为负数会导致向左横移。
     */
    public void strafeToPosition(double inches, double speed) {
        int move = (int) (Math.round(inches * cpi * strafeBias));
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);

        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            telemetry.addData("Working...", " ");
            telemetry.update();
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

    /*
     * 这些函数用于turnWithGyro函数中，以确保输入被正确解释。
     */
    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 360) {
            degrees = degrees - 360;
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 179) {
            degrees = -(360 - degrees);
        }
        return degrees;
    }

    /*
     * 这个函数在程序开始时调用，用于激活IMU集成陀螺仪。
     */
    public void initGyro() {
        // 检查Rev Hub的方向
        // 更多信息见ftc-docs.firstinspires.org
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    /*
     * 这个函数在turnWithGyro函数中使用，用于设置编码器模式并旋转。
     */
    public void turnWithEncoder(double input) {
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }
}
