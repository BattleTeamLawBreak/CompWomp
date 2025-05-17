/*
Copyright 2025 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous

public class AutonLeft extends LinearOpMode {
    private DcMotor frontleftDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backrightDrive = null;
    Servo wrist = null;
    Servo claw = null;
    private DcMotor leftArm;  
    private DcMotor rightArm; 
    IMU imu;
    double kp = 0.4;
    double ki = 0.5;
    double kd = 0.3;
    private ElapsedTime runtime = new ElapsedTime();
    

  static final double HD_COUNTS_PER_REV = 28;
  static final double DRIVE_GEAR_REDUCTION = 17;
  static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
  static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
  static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;


    @Override
    public void runOpMode() {
        frontleftDrive = hardwareMap.get(DcMotor.class, "rearleft");
        frontrightDrive = hardwareMap.get(DcMotor.class, "rearright");
        backleftDrive = hardwareMap.get(DcMotor.class, "frontright");
        backrightDrive = hardwareMap.get(DcMotor.class, "frontleft"); 
        wrist = hardwareMap.get(Servo.class, "servo2");
        claw = hardwareMap.get(Servo.class, "servo3");
        imu = hardwareMap.get(IMU.class, "imu");

 
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
       
        
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftArm = hardwareMap.get(DcMotor.class, "motor3");
        rightArm = hardwareMap.get(DcMotor.class, "motor4");
        
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        
        double flpower, frpower, blpower, brpower;
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        drive = 0.3;
        strafe = 0;
        turn = 0;
        
        frpower = drive - turn + strafe;
        brpower = drive - turn - strafe;
        flpower = drive + turn - strafe;
        blpower = drive + turn + strafe;
        
        frontleftDrive.setPower(flpower);
        frontrightDrive.setPower(frpower);
        backleftDrive.setPower(blpower);
        backrightDrive.setPower(brpower);
        sleep(300);
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        
        wrist.setPosition(.8);
        sleep(1000);
        claw.setPosition(0);
        
        leftArm.setTargetPosition(-1080);
        rightArm.setTargetPosition(-1080);
        leftArm.setPower(.3);
        rightArm.setPower(.3);
        leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(2000);
        
        
        
        claw.setPosition(.5);
        runtime.reset();
        
        
        while (opModeIsActive()) {
            double leftPower;
            double rightPower;
            
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

          
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
            //frontleftDrive.setPower(leftPower);
            //frontrightDrive.setPower(rightPower);

         
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            



               


        }
    }
    public void Moveforward(double distance, double power) {
       
       
    }
    
    public void Turnright() {
        
    }
    
    public void Turnleft() {
        
    }
    
    
    public double PIDcontroller(double TargetPoint, double PositionRobot) {
    
    
        double error = TargetPoint - PositionRobot; 
        error *= kp;
        return error;
    }
    
    

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
    
        // keep in mind that the result is in radians
        return radians;
}

}
