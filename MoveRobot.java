package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MoveRobot{
    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel
    HardwareMap hwMap = null;

    public void initMoveRobot(HardwareMap hwMap){
        this.hwMap = hwMap;

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
}
