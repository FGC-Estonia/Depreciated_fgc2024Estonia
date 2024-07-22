package org.firstinspires.ftc.teamcode;  //place where the code is located


import com.qualcomm.robotcore.hardware.Servo;
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
    }

    public void raise(double joysitck, double rightStick){
        frontElevatorEx.setPower(joysitck + 0.5*rightStick);
        backElevatorEx.setPower(-joysitck - 0.5*rightStick);
    }
}  