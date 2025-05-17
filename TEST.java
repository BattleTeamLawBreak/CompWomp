package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(group = "Autonomous")
public class TEST extends LinearOpMode {

    // Motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx leftArm, rightArm;

    // Servos
    private Servo servo3;  // Wrist
    private Servo servo2;  // Claw

    // IMU
    private BNO055IMU imu;
    private Orientation angles;

    // PID Controller for turning
    private PIDController pidController;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        backLeft   = hardwareMap.get(DcMotorEx.class, "rearleft");
        backRight  = hardwareMap.get(DcMotorEx.class, "rearright");

        leftArm = hardwareMap.get(DcMotorEx.class, "motor3");
        rightArm = hardwareMap.get(DcMotorEx.class, "motor4");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and configure encoders
        resetEncoders();

        // Initialize IMU
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        // Wait for the IMU to calibrate
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Initialize PID Controller for turning
        pidController = new PIDController(0.01, 0.0, 0.0); // Tune these coefficients

        // Set initial servo positions
        servo2.setPosition(0.5);  // Claw
        servo3.setPosition(0.2);  // Wrist

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
             moveArmToPosition(-400, 0.8);
                servo3.setPosition(0.26);
                moveForward(-885, 0.5);
                sleep(70);
                moveArmToPosition(-540, 0.5);
                sleep(75);
                moveBackward(-195, 0.5);
                sleep(75);
                servo2.setPosition(0.8);
                sleep(20);
                moveBackward(-120, 0.5);
                
                
                
                
    
                // Open the claw
              // Release the claw
               
    
    
                // Strafe left (approx 2 feet)
                strafeLeft(-1300, 0.40);  // Adjust tick value as needed
                moveArmToPosition(-1675, 0.8);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                sleep(250);
                moveBackward(-100, 0.5);
                servo2.setPosition(.8);
                
                sleep(250);
                servo2.setPosition(.5);
                sleep(250);
                moveArmToPosition(-485, 0.8);//less=less
                holdArmPosition();
                servo3.setPosition(0.35); //
                sleep(75);
                strafeLeft(1400, 0.50);
                sleep(25);
                moveForward(-650, 0.40);
                sleep(75);
                moveArmToPosition(-548, 0.7);//less=less
                servo3.setPosition(0.30);
                sleep(75);   
                                    
                moveBackward(-250, 0.40);
                
                servo2.setPosition(0.8);
                sleep(75);             
                                        
                 strafeLeft(-1190, 0.50);
                 sleep(250);
                moveForward(-840, 0.6);
                sleep(250);
                strafeLeft(-340, 0.5);
                sleep(250);
                moveBackward(-1175, 0.5);
                moveBackward(1235, 0.5);
                sleep(250);
                strafeLeft(-250, 0.50);
                
                sleep(250);
                moveBackward(-1250, 0.5);
                sleep(250);
                moveForward(-600, 0.4);

                
                
                 moveArmToPosition(-1675, 0.8);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                moveForward(250, 0.35);
                sleep(250);
                servo2.setPosition(.5);
                sleep(125);
                moveArmToPosition(-485, 0.8);
                moveBackward(125, 0.35);//less=less
                holdArmPosition();
                servo3.setPosition(0.35); //
    
                strafeLeft(1805, 0.50);
                sleep(75);
                moveForward(-550, 0.4);
                sleep(75);
                moveArmToPosition(-548, 0.8);//less=less
                servo3.setPosition(0.30);
                sleep(75);   
                                    
                moveBackward(-250, 0.40);
                servo2.setPosition(0.8);
                            
                  
                strafeLeft(-1300, 0.50);  // Adjust tick value as needed
                
                 // Adjusted arm position
                moveArmToPosition(-1675, 0.8);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                sleep(250);
                moveBackward(-150, 0.5);
                servo2.setPosition(.8);
                
                sleep(250);
                servo2.setPosition(.5);
                sleep(250);
                moveArmToPosition(-485, 0.8);//less=less
                holdArmPosition();
                servo3.setPosition(0.35); //
                sleep(75);
                strafeLeft(1400, 0.50);
                sleep(25);
                moveForward(-650, 0.40);
                sleep(75);
                moveArmToPosition(-548, 0.7);//less=less
                servo3.setPosition(0.30);
                sleep(75);   
                                    
                moveBackward(-250, 0.40);
        }
    }

    // Movement methods
    private void moveForward(int ticks, double power) {
        setTargetPosition(ticks);
        setMotorPower(power);
        waitForMotors();
    }

    private void moveBackward(int ticks, double power) {
        setTargetPosition(-ticks);
        setMotorPower(power);
        waitForMotors();
    }

    private void strafeLeft(int ticks, double power) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() - ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(power);
        waitForMotors();
    }

    private void setTargetPosition(int ticks) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void waitForMotors() {
        while (opModeIsActive() &&
               (frontLeft.isBusy() || frontRight.isBusy() ||
                backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Status", "Moving");
            telemetry.update();
        }
        stopMotors();
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // Arm control methods
    private void moveArmToPosition(int position, double power) {
        leftArm.setTargetPosition(position);
        rightArm.setTargetPosition(position);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(power);
        rightArm.setPower(power);

        while (opModeIsActive() && leftArm.isBusy() && rightArm.isBusy()) {
            telemetry.addData("Arm", "Moving to position %d", position);
            telemetry.update();
        }
        holdArmPosition();
    }

    private void holdArmPosition() {
        leftArm.setPower(0.1);
        rightArm.setPower(0.1);
    }

    // IMU methods
    private double getHeading() {
        angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    private void turnToAngle(double targetAngle, double maxPower) {
        double error = targetAngle - getHeading();

        while (opModeIsActive() && Math.abs(error) > 1) {
            double correction = pidController.calculate(targetAngle, getHeading());
            correction = Math.max(-maxPower, Math.min(correction, maxPower));

            frontLeft.setPower(-correction);
            backLeft.setPower(-correction);
            frontRight.setPower(correction);
            backRight.setPower(correction);

            error = targetAngle - getHeading();
            telemetry.addData("Heading", getHeading());
            telemetry.addData("Error", error);
            telemetry.update();
        }

        stopMotors();
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // PID Controller class
    public class PIDController {
        private double kP, kI, kD;
        private double integralSum = 0;
        private double lastError = 0;
        private double lastTime = 0;

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double calculate(double target, double current) {
            double currentTime = System.currentTimeMillis();
            double timeDifference = (currentTime - lastTime) / 1000.0;
            lastTime = currentTime;

            double error = target - current;
            integralSum += error * timeDifference;
            double derivative = (error - lastError) / timeDifference;
            lastError = error;

            return (kP * error) + (kI * integralSum) + (kD * derivative);
        }
    }
}
