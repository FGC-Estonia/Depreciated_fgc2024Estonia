package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Main extends LinearOpMode {
    
    RunMotor runMotor;
    
    @Override
    public void runOpMode() {
        
        runMotor = new RunMotor();
        runMotor.RunMotor(hardwareMap);
        
        waitForStart();
        
        while (opModeIsActive()) {
            runMotor.SetSpeed();
        }
    }
}
