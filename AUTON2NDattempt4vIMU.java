   package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class AUTON2NDattempt4vIMU extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftArm, rightArm;
    private Servo servo3;  // Wrist
    private Servo servo2;  // Claw
    private IMU imu = null;
    
        private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    
    private double          headingError  = 0;
    
        static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03; 
        static final double     DRIVE_SPEED             = 0.1;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;

    
        @Override
        public void runOpMode() {
            // Initialize hardware
            frontLeft  = hardwareMap.get(DcMotor.class, "frontleft");
            frontRight = hardwareMap.get(DcMotor.class, "frontright");
            backLeft   = hardwareMap.get(DcMotor.class, "rearleft");
            backRight  = hardwareMap.get(DcMotor.class, "rearright");

            leftArm = hardwareMap.get(DcMotor.class, "motor3");
            rightArm = hardwareMap.get(DcMotor.class, "motor4");
            servo3 = hardwareMap.get(Servo.class, "servo3");
            servo2 = hardwareMap.get(Servo.class, "servo2");
    
            // Set motor directions
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            leftArm.setDirection(DcMotor.Direction.FORWARD);
            rightArm.setDirection(DcMotor.Direction.REVERSE);
            
           
            // Reset and configure arm encoders
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
             frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));

            imu.resetYaw();
    
            // Set initial servo position
            
            servo2.setPosition(.5);  // Claw
            servo3.setPosition(0.2);// Wrist
    
            telemetry.addData("Status", "Initialized");
            telemetry.update();
    
            waitForStart();
    //remember EVERYTHING IS REVERSED IF IT is -500 FORWARD IS FORWARD MAKE EVERYTIHNG NEGITIVE
            if (opModeIsActive()) {
            
               
                
                                      
                Thread f = new Thread(() -> {
                moveArmToPosition(-400, 0.8);
                servo3.setPosition(0.26);
                 });
                 sleep(25);
            Thread g = new Thread(() -> {
                moveForward(-885, 0.5);
                turnToHeading(0.5, 0);
                 });
        // Start the thread
                f.start();
                g.start();
                sleep(1100);
                moveArmToPosition(-540, 0.5);
                sleep(75);
                moveBackward(-195, 0.5);
                turnToHeading(0.5, 0);
                sleep(75);
                servo2.setPosition(0.8);
                sleep(20);
                moveBackward(-120, 0.5);
                turnToHeading(0.5, 0);
                
                
                
                
    
                // Open the claw
              // Release the claw
               
    
    
                Thread l = new Thread(() -> {
                moveArmToPosition(-400, 0.8);
                strafeLeft(-1300, 0.4);
                 });
                strafeLeft(-1300, 0.4);// Adjust tick value as needed
                Thread h = new Thread(() -> {
                moveArmToPosition(-1675, 0.8);
                holdArmPosition();
                 });
                l.start();
                h.start();
                sleep(2000);
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                sleep(250);
                moveBackward(-110, 0.5);
                turnToHeading(0.5, 0);
                servo2.setPosition(.8);
                
                sleep(250);
                servo2.setPosition(.5);
                sleep(250);
                moveArmToPosition(-485, 0.8);//less=less
                holdArmPosition();
                servo3.setPosition(0.35); //
                sleep(75);
                strafeLeft(1400, 0.50);
                turnToHeading(0.5, 0);
                sleep(25);
                moveForward(-675, 0.40);
                turnToHeading(0.5, 0);
                sleep(75);
                moveArmToPosition(-548, 0.7);//less=less
                servo3.setPosition(0.30);
                sleep(75);   
                                    
                moveBackward(-250, 0.40);
                turnToHeading(0.5, 0);
                
                servo2.setPosition(0.8);
                sleep(75);             
                                        
                 strafeLeft(-1190, 0.50);
                 turnToHeading(0.5, 0);
                 sleep(250);
                moveForward(-840, 0.6);
                turnToHeading(0.5, 0);
                sleep(250);
                strafeLeft(-340, 0.5);
                turnToHeading(0.5, 0);
                sleep(250);
                moveBackward(-1175, 0.5);
                turnToHeading(0.5, 0);
                moveBackward(1235, 0.5);
                turnToHeading(0.5, 0);
                sleep(250);
                strafeLeft(-250, 0.40);
                turnToHeading(0.5, 0);
                sleep(250);
                moveBackward(-1250, 0.5);
                turnToHeading(0.5, 0);
                sleep(250);
                moveForward(-500, 0.4);
                turnToHeading(0.5, 0);

                
                
                 moveArmToPosition(-1675, 0.8);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                moveForward(350, 0.35);
                turnToHeading(0.5, 0);
                sleep(250);
                servo2.setPosition(.5);
                sleep(125);
                moveArmToPosition(-485, 0.8);
                moveBackward(100, 0.35);
                turnToHeading(0.5, 0);//less=less
                holdArmPosition();
                servo3.setPosition(0.35); //
    
                strafeLeft(1805, 0.50);
                turnToHeading(0.5, 0);
                
                sleep(75);
                moveForward(-550, 0.4);
                turnToHeading(0.5, 0);
                sleep(75);
                moveArmToPosition(-548, 0.8);//less=less
                servo3.setPosition(0.30);
                sleep(75);   
                                    
                moveBackward(-250, 0.40);
                turnToHeading(0.5, 0);
                servo2.setPosition(0.8);
                            
                  
                strafeLeft(-1300, 0.50);
                turnToHeading(0.5, 0);// Adjust tick value as needed
                
                 // Adjusted arm position
                moveArmToPosition(-1675, 0.8);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                sleep(250);
                moveBackward(-150, 0.5);
                turnToHeading(0.5, 0);
                servo2.setPosition(.8);
                
                sleep(250);
                servo2.setPosition(.5);
                sleep(250);
                moveArmToPosition(-485, 0.8);//less=less
                holdArmPosition();
                servo3.setPosition(0.35); //
                sleep(75);
                strafeLeft(1400, 0.50);
                turnToHeading(0.5, 0);
                sleep(25);
                moveForward(-650, 0.40);
                turnToHeading(0.5, 0);
                sleep(75);
                moveArmToPosition(-548, 0.7);//less=less
                servo3.setPosition(0.30);
                sleep(75);   
                                    
                moveBackward(-250, 0.40);
                turnToHeading(0.5, 0);
                
                
            }

            while(opModeIsActive()) {
                idle();
            }
        }
    
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
            while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                telemetry.addData("Moving", "Motors running...");
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
        
        private void moveArmToPositionSkip(int position, double power) {
            leftArm.setTargetPosition(position);
            rightArm.setTargetPosition(position);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftArm.setPower(power);
            rightArm.setPower(power);
    
            // while (opModeIsActive() && leftArm.isBusy() && rightArm.isBusy()) {
            //     telemetry.addData("Arm", "Moving to position %d", position);
            //     telemetry.update();
            // }
            holdArmPosition();
        }
    
        private void holdArmPosition() {
            leftArm.setPower(0.1);
            rightArm.setPower(0.1);
        }

        public void turnToHeading(double maxTurnSpeed, double heading) {

            // Run getSteeringCorrection() once to pre-calculate the current error
            getSteeringCorrection(heading, P_DRIVE_GAIN);

            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

                // Clip the speed to the maximum permitted value.
                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

                // Pivot in place by applying the turning correction
                moveRobot(0, turnSpeed);
            }

            // Stop all motion;
            moveRobot(0, 0);
        }

        public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
            targetHeading = desiredHeading;  // Save for telemetry

            // Determine the heading current error
            headingError = targetHeading - getHeading();

            // Normalize the error to be within +/- 180 degrees
            while (headingError > 180)  headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
            return Range.clip(headingError * proportionalGain, -1, 1);
        }

        public void moveRobot(double drive, double turn) {
            driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
            turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

            leftSpeed  = drive - turn;
            rightSpeed = drive + turn;

            // Scale speeds down if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

//frontLeft, frontRight, backLeft, backRight;
            frontLeft.setPower(leftSpeed);
            frontRight.setPower(rightSpeed);
        }

        public double getHeading() {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);
        }
}
