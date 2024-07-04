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
        private double lastSpeedLeftFrontDrive=0;
        private double lastSpeedLeftBackDrive=0;
        private double lastSpeedRightBackDrive=0;
        private double lastSpeedRightFrontDrive=0;
        private final double accelerationLimit=20; //RPSS revolutions per second  second 
        boolean TractionControlstatus = false;

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

            gamepad1.rumble(0.5, 0.5, 1000);
            gamepad2.rumble(0.5, 0.5, 1000);
            
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

                if (gamepad1.a && TractionControlstatus) {
                    TractionControlstatus = false;
                } else if (gamepad1.a && !TractionControlstatus) {
                    TractionControlstatus = true;
                }
                
                // Get the acceleration data from the IMU

                // Display acceleration data on telemetry
                telemetry.addData("gyro", imu.getRobotYawPitchRollAngles());
                telemetry.addData("leftFronDrive", ((leftFrontDriveEx.getVelocity())));
                telemetry.update();

                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                if (!gamepad1.left_bumper) { // Field centric drive when in manual mode (autodrive button not pressed)
                    double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    double x = drive * Math.cos(heading) - strafe * Math.sin(heading);
                    double y = drive * Math.sin(heading) + strafe * Math.cos(heading);

                    moveRobot(x, y, turn);
                }else { // Inputs for automatic mode driving
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
            
            // Calculate wheel speeds normalized to the wheels.
            double leftFrontRawSpeed = (leftFrontPowerRaw/max*maxradian/1.2039);
            double leftBackRawSpeed = (leftBackPowerRaw/max*maxradian/1.2039);
            double rightFrontRawSpeed = (rightFrontPowerRaw/max*maxradian);
            double rightBackRawSpeed = (rightBackPowerRaw/max*maxradian);
            if (TractionControlstatus) {
                tractionControl(leftFrontRawSpeed, leftBackRawSpeed, rightFrontRawSpeed, rightBackRawSpeed);
            } else {
                leftBackDriveEx.setVelocity(leftBackRawSpeed);
                leftFrontDriveEx.setVelocity(leftFrontRawSpeed);
                rightBackDriveEx.setVelocity(rightBackRawSpeed);
                rightFrontDriveEx.setVelocity(rightFrontRawSpeed);
            }

        }
        private void tractionControl(double speedLeftFrontDrive, double speedLeftBackDrive, double speedRightBackDrive, double speedRightFrontDrive){
            double deltaTime=System.currentTimeMillis();
            double deltaSpeedLeftFrontDrive=speedLeftFrontDrive;
            double deltaSpeedLeftBackDrive=speedLeftBackDrive;
            double deltaSpeedRightFrontDrive=speedRightFrontDrive;
            double deltaSpeedRightBackDrive=speedRightBackDrive;
            double maxDelta=deltaTime*accelerationLimit;
            double maxMultiplier=1;

            if (Math.abs(deltaSpeedLeftBackDrive/maxDelta)>Math.abs(maxMultiplier)){
                maxMultiplier=deltaSpeedLeftBackDrive/maxDelta;
            } if (Math.abs(deltaSpeedLeftFrontDrive/maxDelta)>Math.abs(maxMultiplier)){
                maxMultiplier=deltaSpeedLeftFrontDrive/maxDelta;
            } if (Math.abs(deltaSpeedRightBackDrive/maxDelta)>Math.abs(maxMultiplier)){
                maxMultiplier=deltaSpeedRightBackDrive/maxDelta;
            } if (Math.abs(deltaSpeedRightFrontDrive/maxDelta)>Math.abs(maxMultiplier)){
                maxMultiplier=deltaSpeedRightFrontDrive/maxDelta;
            }

            leftBackDriveEx.setVelocity(speedLeftBackDrive-deltaSpeedLeftBackDrive+deltaSpeedLeftBackDrive/maxMultiplier);
            leftFrontDriveEx.setVelocity(speedLeftFrontDrive-deltaSpeedLeftFrontDrive+deltaSpeedLeftFrontDrive/maxMultiplier);
            rightBackDriveEx.setVelocity(speedRightBackDrive-deltaSpeedRightBackDrive+deltaSpeedRightBackDrive/maxMultiplier);
            rightFrontDriveEx.setVelocity(speedRightFrontDrive-deltaSpeedRightFrontDrive+deltaSpeedRightFrontDrive/maxMultiplier);

            lastTime=System.currentTimeMillis();
            lastSpeedLeftFrontDrive=speedLeftFrontDrive;
            lastSpeedLeftBackDrive=speedLeftBackDrive;
            lastSpeedRightBackDrive=speedRightBackDrive;
            lastSpeedRightFrontDrive=speedRightFrontDrive;

            telemetry.addData("left back wanted speed",speedLeftBackDrive);
            telemetry.addData("left back got speed",speedLeftBackDrive-deltaSpeedLeftBackDrive+deltaSpeedLeftBackDrive/maxMultiplier);
            telemetry.addData("left front wanted speed",speedLeftFrontDrive);
            telemetry.addData("left front got speed",speedLeftFrontDrive-deltaSpeedLeftFrontDrive+deltaSpeedLeftFrontDrive/maxMultiplier);

        }
    }