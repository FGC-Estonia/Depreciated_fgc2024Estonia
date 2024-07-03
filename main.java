import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    private IMU imu;

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

    private DcMotorEx leftFrontDriveEx   = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx  = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx    = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx   = null;  //  Used to control the right back drive wheel

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
            leftFrontDriveEx  = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_CH");
            rightFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
            leftBackDriveEx  = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_CH");
            rightBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");
    
            leftFrontDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
            leftBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
            rightFrontDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
            rightBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
        }


        
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the start button to be pressed
        imu.resetYaw();
        waitForStart();

        while (opModeIsActive()) {
            // Get the acceleration data from the IMU

            // Display acceleration data on telemetry
            telemetry.addData("gyro", imu.getRobotYawPitchRollAngles());
            telemetry.addData("leftFronDrive", ((leftFrontDriveEx.getVelocity())));
            telemetry.update();

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (!gamepad1.left_bumper) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2;
                double x = drive * Math.cos(heading) - strafe * Math.sin(heading);
                double y = drive * Math.sin(heading) + strafe * Math.cos(heading);

                moveRobot(x, y, turn);
            }else {
                moveRobot(drive, strafe, turn);
            }
        }
    }
    public void moveRobot(double drive, double strafe, double turn) {
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
        
        // Send powers to the wheels.
        leftFrontDriveEx.setVelocity(leftFrontPowerRaw/max*maxradian/1.2039);
        leftBackDriveEx.setVelocity(leftBackPowerRaw/max*maxradian/1.2039);
        rightFrontDriveEx.setVelocity(rightFrontPowerRaw/max*maxradian);
        rightBackDriveEx.setVelocity(rightBackPowerRaw/max*maxradian);
    }
    private void tractionControl(double powLeftFrontDrive, double powLeftBackDrive, double powRightBackDrive, double powRightLeftDrive, double currentTime){


        lastTime=System.currentTimeMillis();

        //lastPowerLeftFrontDrive=LeftFrontDriveEx.getVelocity();
        //lastPowerLeftBackDrive=LeftBackDriveEx.getPo;
        lastPowerRightBackDrive=powRightBackDrive;
        lastPowerRightLeftDrive=powRightLeftDrive;
        lastSpeedLeftFrontDrive=0;
        lastSpeedLeftBackDrive=0;
        lastSpeedRightBackDrive=0;
        lastSpeedRightLeftDrive=0;
    }
}