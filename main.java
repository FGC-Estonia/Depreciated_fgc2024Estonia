package org.firstinspires.ftc.teamcode; 

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //type of rev code. opmode is also possible

public class Main extends LinearOpMode { //file name is Main.java    extends the prebuilt LinearOpMode by rev to run
    /*
    * Import external classes 
    * class name  name to be used in this class; 
    *eg:
    * RunMotor runMotor;
    */
    
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

        waitForStart();
        
        while (opModeIsActive()) {
            //runMotor.SetSpeed();

        }
    }
}
