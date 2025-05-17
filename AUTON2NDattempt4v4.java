   package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AUTON2NDattempt4v4 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftArm, rightArm;
    private Servo servo3;  // Wrist
    private Servo servo2;  // Claw

    
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
                 });
        // Start the thread
                f.start();
                g.start();
                sleep(1100);
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
                strafeLeft(-1300, 0.4);  // Adjust tick value as needed
                moveArmToPosition(-1675, 0.8);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                sleep(250);
                moveBackward(-120, 0.5);
                servo2.setPosition(.8);
                
                sleep(250);
                servo2.setPosition(.5);
                sleep(250);
                moveArmToPosition(-485, 0.8);//less=less
                holdArmPosition();
                servo3.setPosition(0.35); //
                sleep(75);
                strafeLeft(1385, 0.50);
                sleep(25);
                moveForward(-670, 0.40);
                sleep(75);
                moveArmToPosition(-548, 0.7);//less=less
                servo3.setPosition(0.30);
                sleep(75);   
                                    
                moveBackward(-250, 0.40);
                
                servo2.setPosition(0.8);
                sleep(75);             
                                        
                 strafeLeft(-1215, 0.50);
                 sleep(250);
                moveForward(-840, 0.6);
                sleep(250);
                strafeLeft(-360, 0.5);
                sleep(250);
                moveBackward(-1275, 0.5);
                moveBackward(1135, 0.5);
                sleep(250);
                strafeLeft(-225, 0.40);
                sleep(250);
                moveBackward(-1250, 0.5);
                sleep(250);
                moveForward(-500, 0.4);

                
                
                 moveArmToPosition(-1675, 0.8);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                moveForward(350 ,0.35);
                sleep(250);
                servo2.setPosition(.5);
                sleep(125);
                moveArmToPosition(-485, 0.6);
                moveBackward(100, 0.35);//less=less
                holdArmPosition();
                servo3.setPosition(0.35); //
    
                strafeLeft(1790, 0.50);
                sleep(75);
                moveForward(-570, 0.4);
                sleep(75);
                moveArmToPosition(-548, 0.8);//less=less
                servo3.setPosition(0.30);
                sleep(75);   
                                    
                moveBackward(-250, 0.40);
                servo2.setPosition(0.8);
                sleep(75);
                  
                strafeLeft(-1300, 0.50);  // Adjust tick value as needed
                
                 // Adjusted arm position
                moveArmToPosition(-1675, 0.8);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.5);
                servo2.setPosition(.8);
                sleep(250);
                moveBackward(-250, 0.5);
                servo2.setPosition(.8);
                
                sleep(250);
                servo2.setPosition(.5);
                sleep(250);
                moveArmToPosition(-485, 0.6);//less=less
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
    }
