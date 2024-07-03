import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "Motor_Port_0_CH");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Motor_Port_2_CH");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "Motor_Port_1_CH");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Motor_Port_3_CH");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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

            drive = -gamepad1.left_stick_y
            strafe = gamepad1.left_stick_x
            turn = gamepad1.right_stick_x

            moveRobot(drive, strafe, turn)
        }

    public void moveRobot(double drive, double strafe, double turn) {

    }
    }
}