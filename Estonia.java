package org.firstinspires.ftc.teamcode; 

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //type of rev code. opmode is also possible
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main code Estonia")
public class Estonia extends LinearOpMode { //file name is Main.java    extends the prebuilt LinearOpMode by rev to run
    /*
    * Import external classes 
    * class name  name to be used in this class; 
    *eg:
    * RunMotor runMotor;
    */
    MoveRobot moveRobot;
    @Override
    public void runOpMode() {
        /*
         * map objects
         * name to be used = new class()
         * eg:
         * runMotor = new RunMotor();
         * 
         * if te external classes require initialisation do it here
         * eg:
         * runMotor.initRunMotor(hardwareMap);
         */
         moveRobot =new MoveRobot();
         moveRobot.initMoveRobot(hardwareMap, telemetry);

        waitForStart();
        
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            moveRobot.move(drive, strafe, turn, true);

        }
    }
}
