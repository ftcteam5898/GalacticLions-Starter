package org.firstinspires.ftc.team18443;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Auto_BlueLeft", group="Auto Blue", preselectTeleOp="StraferTeleOp")
public class Auto_BlueLeft extends LinearOpMode {
    // variable declaration & setup
    DcMotor frontLeft, frontRight, backLeft, backRight, arm;
    Servo claw, stupid, intakeWrist;


    // motor counts per rotation (ticks/pulses per rotation)
    // check motor specs from manufacturer
    // 537.7 is for GoBilda 312 RPM Yellow Jacket motor
    double cpr = 537.7;

    // adjust gearRatio if you have geared up or down your motors
    double gearRatio = 1;

    // wheel diameter in inches
    // 3.779 is for the GoBilda mecanum wheels
    double diameter = 3.779;

    // counts per inch: cpr * gear ratio / (pi * diameter (in inches))
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);

    // use calibrate auto to check this number before proceeding
    double bias = 0.94; // adjust based on calibration opMode

    double strafeBias = 0.9; // change to adjust only strafing movement
    //
    double conversion = cpi * bias;
    //
    IMU imu;

    @Override
    public void runOpMode(){

        initGyro();

        // setup motors
        frontLeft = hardwareMap.dcMotor.get("lf");
        frontRight = hardwareMap.dcMotor.get("rf");
        backLeft = hardwareMap.dcMotor.get("lb");
        backRight = hardwareMap.dcMotor.get("rb");

        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // setup servos
        claw = hardwareMap.servo.get("claw");
        stupid = hardwareMap.get(Servo.class, "stupid");
        intakeWrist = hardwareMap.get(Servo.class, "Intake Wrist");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set starting positions of claw
        closeClaw();
        // please delete stupid once we remove the stupid servo.
        stupid.setPosition(1);
        intakeWrist.setPosition(0);
        sleep(500);

        // wait for Start to be pressed
        waitForStart();

        // Call functions here

        // Begin with Blue specimen
        // Extend arm and move forward; attach specimen
        // forward(25,.5);
        // sleep(500);
        extendArm(-2850);
        forward(38,.6);
        sleep(500);
        lowerArm(2000);
        openClaw();

        // Reverse and strafe to Observation Zone
        back(20,.4);
        turnRight(90, .4);
        claw.setPosition(0.65);
        sleep(200);
        lowerArm(600);
        forward(40,.4);
        strafeRight(20, .4);
        strafeLeft(5.25,.4);
        openClaw();
        forward(26,.4);
        sleep(200);
        claw.setPosition(0.65);
        sleep(300);
        back(60,.4);
        turnLeft(90, .3);
        forward(18,.5);
        sleep(500);
        extendArm(-2800);
        sleep(200);
        forward(9,.6);
        sleep(500);
        lowerArm(2100);
        sleep(500);
        openClaw();
        back(5,.3);
        back(24, .7);
        strafeRight(45, .7);
        lowerArm(700);
        //strafeRight(47,.5);
        //lowerArm(500);

        // Go forward, move right, push blue sample into ob zone
        //forward(30,.2);
        //sleep(500);
        //strafeRight(10,.3);
        //back(48,.4);
        // Do it again for second sample
        //forward(46,.3);
        //strafeRight(11,.3);
        //back(44,.3);

        // do it again for third sample
        //forward(42,.3);
        //strafeRight(18,.3);
        //strafeLeft(1,.2);
        //back(50,.5);
        //lowerArm(600);

        //sleep(1000);
    }






    /**
     * Use to make the robot go forward a number of inches
     * @param inches distance to travel in inches
     * @param speed has a range of [0,1]
     */
    public void forward(double inches, double speed){ moveToPosition(inches, speed); }

    /**
     * Use to make the robot go backward a number of inches
     * @param inches distance to travel in inches
     * @param speed has a range of [0,1]
     */
    public void back(double inches, double speed){ moveToPosition(-inches, speed); }

    /**
     Rotate the robot left
     @param degrees the amount of degrees to rotate
     @param speed has a range of [0,1]
     */
    public void turnLeft(double degrees, double speed){ turnWithGyro(degrees, -speed); }

    /**
     Rotate the robot right
     @param degrees the amount of degrees to rotate
     @param speed has a range of [0,1]
     */
    public void turnRight(double degrees, double speed){ turnWithGyro(degrees, speed); }

    /**
     Strafe left
     @param inches the distance in inches to strafe
     @param speed has a range of [0,1]
     */
    public void strafeLeft(double inches, double speed){ strafeToPosition(-inches, speed); }

    /**
     Strafe right
     @param inches the distance in inches to strafe
     @param speed has a range of [0,1]
     */
    public void strafeRight(double inches, double speed){ strafeToPosition(inches, speed); }

    /**
     Extends the arm
     @param ticks the number of ticks the motor needs to move
     */
    public void extendArm(int ticks){
        arm.setTargetPosition(arm.getCurrentPosition() + ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        while (arm.isBusy()){
            telemetry.addData("Busy...", "");
            telemetry.update();
        }
        arm.setPower(0);
    }

    /**
     Closes the arm
     @param ticks the number of ticks the motor needs to move
     */
    public void lowerArm(int ticks){
        arm.setTargetPosition(arm.getCurrentPosition() + ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        while (arm.isBusy()){
            telemetry.addData("Busy...", "");
            telemetry.update();
        }
        arm.setPower(0);
    }

    /**
     Opens the claw on the arm of the robot
     */
    public void openClaw() {
        claw.setPosition(0.5);
    }

    /**
     Closes the claw on the arm of the robot
     */
    public void closeClaw() {
        claw.setPosition(0.65);
    }

   /**
     This function's purpose is simply to drive forward or backward.
     To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed){
        int move = (int)(Math.round(inches*conversion));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Busy...", "");
            telemetry.update();
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    /**
     This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number
     of degrees (+/- 2). Degrees should always be positive, make speedDirection negative
     to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection) {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = -orientation.getYaw(AngleUnit.DEGREES);//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //

        double first;
        double second;
        //
        if (speedDirection > 0){//set target positions
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
        }else{
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //

        }
        //

        double firsta = convertify(first - 2);//178
        double firstb = convertify(first + 2);//-178
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                orientation = imu.getRobotYawPitchRollAngles();
                yaw = -orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                orientation = imu.getRobotYawPitchRollAngles();
                yaw = -orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        double seconda = convertify(second - 2);//178
        double secondb = convertify(second + 2);//-178
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                orientation = imu.getRobotYawPitchRollAngles();
                yaw = -orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                orientation = imu.getRobotYawPitchRollAngles();
                yaw = -orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        //
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     These functions are used in the turnWithGyro function to ensure inputs
     are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 360){
            degrees = degrees - 360;
        }
        else if(degrees < -180){
            degrees = 360 + degrees;
        }
        else if(degrees > 179){
            degrees = -(360 - degrees);
        }
        return degrees;
    }

    /**
     This function uses the encoders to strafe left or right.
     Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        int move = (int)(Math.round(inches * cpi * strafeBias));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Working...", " ");
            telemetry.update();}
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    /**
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        // Check the orientation of the Rev Hub
        // more info on ftc-docs.firstinspires.org
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    /**
     This function is used in the turnWithGyro function to set the
     encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontLeft.setPower(input);
        backLeft.setPower(input);
        frontRight.setPower(-input);
        backRight.setPower(-input);
    }

}
