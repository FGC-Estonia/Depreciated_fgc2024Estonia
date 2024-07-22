package org.firstinspires.ftc.teamcode;  //place where the code is located


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Erection{
    private Servo leftServo;
    private Servo rightServo;

    private DcMotorEx frontElevatorEx;
    private DcMotorEx backElevatorEx;

    Telemetry telemetry;
    HardwareMap hardwareMap;

    public void initErection(HardwareMap hardwareMapPorted, Telemetry telemetryPorted){
        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;

        frontElevatorEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_EH");
        backElevatorEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_EH");

        frontElevatorEx.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backElevatorEx.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontElevatorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backElevatorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontElevatorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backElevatorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void raise(double joysitck, double rightStick, boolean height100){
        frontElevatorEx.setPower(joysitck + 0.5*rightStick);
        backElevatorEx.setPower(-joysitck - 0.5*rightStick);

        if (height100){
            frontElevatorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backElevatorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontElevatorEx.setTargetPosition(1486);//1000(height mm)/(6mm(hex shaft diameter)*3,14)*28(ticks per rotation)
            backElevatorEx.setTargetPosition(1486);
            backElevatorEx.setPower(1);
            frontElevatorEx.setPower(1);
        } else {
            frontElevatorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backElevatorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("motor position", backElevatorEx.getCurrentPosition());

    }
}  