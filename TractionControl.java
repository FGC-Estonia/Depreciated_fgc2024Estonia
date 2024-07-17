package org.firstinspires.ftc.teamcode;  //place where the code is located

import com.qualcomm.robotcore.hardware.HardwareMap;


public class TractionControl{

    private double lastTime = 0; // create last varriables for traction control
    private double lastSpeedLeftFrontDrive = 0;
    private double lastSpeedLeftBackDrive = 0;
    private double lastSpeedRightBackDrive = 0;
    private double lastSpeedRightFrontDrive = 0;
    private final double accelerationLimit = 20; //RPSS revolutions per second  second 

    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public void initTractionControl(HardwareMap hardwareMapPorted, Telemetry telemetryPorted){
        
        hardwareMap=hardwareMapPorted;

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

    public void avoidSlip(double speedLeftBackDrive, double speedLeftFrontDrive, double speedRightBackDrive, double speedRightFrontDrive, hardwareMap){
        double deltaTime = System.currentTimeMillis() - lastTime;
        double deltaSpeedLeftFrontDrive = speedLeftFrontDrive - speedLeftFrontDrive;
        double deltaSpeedLeftBackDrive = speedLeftBackDrive - speedLeftBackDrive;
        double deltaSpeedRightFrontDrive = speedRightFrontDrive - speedRightFrontDrive;
        double deltaSpeedRightBackDrive = speedRightBackDrive - speedRightBackDrive;
        double maxDelta = deltaTime * accelerationLimit;
        double maxMultiplier = 1;

        if (Math.abs(deltaSpeedLeftBackDrive / maxDelta) > Math.abs(maxMultiplier)) {
            maxMultiplier = deltaSpeedLeftBackDrive / maxDelta;
        }
        if (Math.abs(deltaSpeedLeftFrontDrive / maxDelta) > Math.abs(maxMultiplier)) {
            maxMultiplier = deltaSpeedLeftFrontDrive / maxDelta;
        }
        if (Math.abs(deltaSpeedRightBackDrive / maxDelta) > Math.abs(maxMultiplier)) {
            maxMultiplier = deltaSpeedRightBackDrive / maxDelta;
        }
        if (Math.abs(deltaSpeedRightFrontDrive / maxDelta) > Math.abs(maxMultiplier)) {
            maxMultiplier = deltaSpeedRightFrontDrive / maxDelta;
        }

        leftBackDriveEx.setVelocity(speedLeftBackDrive - deltaSpeedLeftBackDrive + deltaSpeedLeftBackDrive / maxMultiplier);
        leftFrontDriveEx.setVelocity(speedLeftFrontDrive - deltaSpeedLeftFrontDrive + deltaSpeedLeftFrontDrive / maxMultiplier);
        rightBackDriveEx.setVelocity(speedRightBackDrive - deltaSpeedRightBackDrive + deltaSpeedRightBackDrive / maxMultiplier);
        rightFrontDriveEx.setVelocity(speedRightFrontDrive - deltaSpeedRightFrontDrive + deltaSpeedRightFrontDrive / maxMultiplier);

        lastTime = System.currentTimeMillis();
        lastSpeedLeftFrontDrive = speedLeftFrontDrive;
        lastSpeedLeftBackDrive = speedLeftBackDrive;
        lastSpeedRightBackDrive = speedRightBackDrive;
        lastSpeedRightFrontDrive = speedRightFrontDrive;
    }
}