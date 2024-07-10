package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class MoveRobot{
    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel
    
    private HardwareMap hwMap = null;
    private IMU imu;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
  //  private AprilTagTracker aprilTagTracker;

    public void initMoveRobot(HardwareMap hardwareMapPorted, Telemetry telemetryPorted){
        
        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;
       // aprilTagTracker = new AprilTagTracker(hardwareMap, telemetry);
        //aprilTagTracker.initAprilTag(hardwareMap, telemetry);


        // Initializing imu
        imu = hardwareMap.get(IMU.class, "imu");
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        // Mapping motors
        rightFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_CH");
        leftFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_CH");
        leftBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
        rightBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");

        leftFrontDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setSpeed(){
        telemetry.addData("gyro", imu.getRobotYawPitchRollAngles());
        telemetry.update();
        
    }

    public void move(double drive, double strafe, double turn, boolean fieldcentric) {
        double x;
        double y;
        
        if (fieldcentric) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            x = drive * Math.cos(heading) - strafe * Math.sin(heading);
            y = drive * Math.sin(heading) + strafe * Math.cos(heading);
        } else {
            x = drive;
            y = strafe;
        }

        // Calculates raw power to motors
        double leftFrontPowerRaw = x + y + turn;
        double leftBackPowerRaw = x - y + turn;
        double rightFrontPowerRaw = x - y - turn;
        double rightBackPowerRaw = x + y - turn;

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
