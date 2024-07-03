import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    private BNO055IMU imu;

    private double lastTime=0; // create last varriables for traction control
    private double lastPowerLeftFrontDrive=0;
    private double lastPowerLeftBackDrive=0;
    private double lastPowerRightBackDrive=0;
    private double lastPowerRightLeftDrive=0;
    private double lastSpeedLeftFrontDrive=0;
    private double lastSpeedLeftBackDrive=0;
    private double lastSpeedRightBackDrive=0;
    private double lastSpeedRightLeftDrive=0;
    private final double accelerationLimit=200; //RPSS revolutions per second  second 

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    @Override
    public void runOpMode() {


        
        
        
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "Motor_Port_0_CH");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Motor_Port_2_CH");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "Motor_Port_1_CH");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Motor_Port_3_CH");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        if (true){
            if (leftFrontDrive instanceof DcMotorEx && leftBackDrive instanceof DcMotorEx  &&
             rightBackDrive instanceof DcMotorEx && rightBackDrive instanceof DcMotorEx) {
                // Perform the cast
                DcMotorEx leftFrontDriveEx = (DcMotorEx)leftFrontDrive;
                DcMotorEx rightFrontDriveEx = (DcMotorEx)rightFrontDrive;
                DcMotorEx leftBackDriveEx = (DcMotorEx)leftBackDrive;
                DcMotorEx rightBackDriveEx = (DcMotorEx)rightBackDrive;
            
                // Now you can use the enhanced functionality of DcMotorEx
                int tractionControlStatus=0;
            } else {
                int tractionControlStatus=1;
            }
        }


        // Initialize the hardware variables
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up the parameters for the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Initialize the IMU
        imu.initialize(parameters);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Get the acceleration data from the IMU
            Acceleration acceleration = imu.getAcceleration();
            
            // Display acceleration data on telemetry
            telemetry.addData("X", acceleration.xAccel);
            telemetry.addData("Y", acceleration.yAccel);
            telemetry.addData("Z", acceleration.zAccel);
            telemetry.update();

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;



            moveRobot(drive, strafe, turn);
        }
    }
    public void moveRobot(double drive, double strafe, double turn) {
        // Calculates raw power to motors
        double leftFrontPowerRaw = -drive + strafe - turn;
        double leftBackPowerRaw = drive + strafe - turn;
        double rightFrontPowerRaw = drive + strafe + turn;
        double rightBackPowerRaw = -drive + strafe + turn;

        // Calculate the maximum absolute power value for normalization
        double maxRawPower = Math.max(Math.max(Math.abs(leftFrontPowerRaw), Math.abs(leftBackPowerRaw)),
        Math.max(Math.abs(rightFrontPowerRaw), Math.abs(rightBackPowerRaw)));

        double max = Math.max(maxRawPower, 1.0);
        
        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPowerRaw/max/1.2039);
        leftBackDrive.setPower(leftBackPowerRaw/max/1.2039);
        rightFrontDrive.setPower(rightFrontPowerRaw/max);
        rightBackDrive.setPower(rightBackPowerRaw/max);
    }
    private void tractionControl(double powLeftFrontDrive, double powLeftBackDrive, double powRightBackDrive, double powRightLeftDrive, double currentTime){


        lastTime=System.currentTimeMillis();
        lastPowerLeftFrontDrive=powLeftFrontDrive;
        lastPowerLeftBackDrive=powLeftBackDrive;
        lastPowerRightBackDrive=powRightBackDrive;
        lastPowerRightLeftDrive=powRightLeftDrive;
        lastSpeedLeftFrontDrive=0;
        lastSpeedLeftBackDrive=0;
        lastSpeedRightBackDrive=0;
        lastSpeedRightLeftDrive=0;
    }
}