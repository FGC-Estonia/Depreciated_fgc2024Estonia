package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class MoveRobot{
    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel
    HardwareMap hwMap = null;

    private IMU imu;

    public void initMoveRobot(HardwareMap hwMap){
        this.hwMap = hwMap;

            // Initializing imu
            imu = hwMap.get(IMU.class, "imu");
            
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            imu.resetYaw();

            // Mapping motors
            rightFrontDriveEx = hwMap.get(DcMotorEx.class, "Motor_Port_0_CH");
            leftFrontDriveEx = hwMap.get(DcMotorEx.class, "Motor_Port_1_CH");
            leftBackDriveEx = hwMap.get(DcMotorEx.class, "Motor_Port_2_CH");
            rightBackDriveEx = hwMap.get(DcMotorEx.class, "Motor_Port_3_CH");

            leftFrontDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
            leftBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
            rightFrontDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
            rightBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setSpeed(){
        leftBackDriveEx.setPower(1);
    }

    public void move(double drive, double strafe, double turn, boolean fieldcentric) {
        if (fieldcentric) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x = drive * Math.cos(heading) - strafe * Math.sin(heading);
            double y = drive * Math.sin(heading) + strafe * Math.cos(heading);

            drive = x;
            turn = y;
        }

        // Calculates raw power to motors
        double leftFrontPowerRaw = drive + strafe + turn;
        double leftBackPowerRaw = drive - strafe + turn;
        double rightFrontPowerRaw = drive + strafe - turn;
        double rightBackPowerRaw = drive - strafe - turn;

        // Calculate the maximum absolute power value for normalization
        double maxRawPower = Math.max(Math.max(Math.abs(leftFrontPowerRaw), Math.abs(leftBackPowerRaw)),
                Math.max(Math.abs(rightFrontPowerRaw), Math.abs(rightBackPowerRaw)));

        double max = Math.max(maxRawPower, 1.0);
        double maxradian = 1972.92;

        // Calculate wheel speeds normalized to the wheels.
        double leftFrontRawSpeed = (leftFrontPowerRaw / max * maxradian / 1.2039);
        double leftBackRawSpeed = (leftBackPowerRaw / max * maxradian);
        double rightFrontRawSpeed = (rightFrontPowerRaw / max * maxradian / 1.2039);
        double rightBackRawSpeed = (rightBackPowerRaw / max * maxradian);
        
        // Make wheels go speed
        leftBackDriveEx.setVelocity(leftBackRawSpeed);
        leftFrontDriveEx.setVelocity(leftFrontRawSpeed);
        rightBackDriveEx.setVelocity(rightBackRawSpeed);
        rightFrontDriveEx.setVelocity(rightFrontRawSpeed);
    }
}
