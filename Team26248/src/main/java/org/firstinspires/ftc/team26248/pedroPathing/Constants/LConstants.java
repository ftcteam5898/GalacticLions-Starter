package org.firstinspires.ftc.team26248.pedroPathing.Constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
//        ThreeWheelIMUConstants.forwardTicksToInches = 0.001245974390827444;
//        ThreeWheelIMUConstants.strafeTicksToInches = 0.0012461265430305003;
//        ThreeWheelIMUConstants.turnTicksToInches = .001989436789;
//
//        //TODO: Needs to be tested
//        ThreeWheelIMUConstants.leftY = 1.875;
//        ThreeWheelIMUConstants.rightY = -2.5;
//        ThreeWheelIMUConstants.strafeX = -6.375;
//
//        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "br";
//        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "yl";
//        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "xo";
//
//        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
//        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
//        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
//
//        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
//        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        PinpointConstants.hardwareMapName = "pinpoint";

        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        PinpointConstants.strafeX = -6.375;
        PinpointConstants.forwardY = -2.5;

    }
}
