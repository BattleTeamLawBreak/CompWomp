// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
// import com.qualcomm.robotcore.hardware.Servo;
// 
//  
// 
// 
// 
// 
// /*
//  * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
//  * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
//  * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
//  * class is instantiated on the Robot Controller and executed.
//  *
//  * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
//  * It includes all the skeletal structure that all linear OpModes contain.
//  *
//  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
//  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
//  */
// 
// @TeleOp(name = "Team3", group = "LinearOpMode")
// 
// public class Teleop extends LinearOpMode {
//     private DcMotor frontleftDrive;
//     private DcMotor backleftDrive = null;
//     private DcMotor frontrightDrive = null;
//     private DcMotor backrightDrive = null;
//     Servo servo3 = null;
//     Servo servo2 = null;
//     boolean open = true;
//     double tgtPower = 0;
//     private DcMotor leftArm;  
//     private DcMotor rightArm; 
//     private double leftArmPower = 0; 
//     private double rightArmPower = 0; 
//         
//     
//     
// 
//     @Override
//     public void runOpMode() {
//         frontleftDrive = hardwareMap.get(DcMotor.class, "rearleft");
//         frontrightDrive = hardwareMap.get(DcMotor.class, "rearright");
//         backleftDrive = hardwareMap.get(DcMotor.class, "frontright");
//         backrightDrive = hardwareMap.get(DcMotor.class, "frontleft"); 
//         servo3 = hardwareMap.get(Servo.class, "servo3");
//         servo2 = hardwareMap.get(Servo.class, "servo2");
//        
//         
//         frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
//         frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
//         backleftDrive.setDirection(DcMotor.Direction.REVERSE);
//         backrightDrive.setDirection(DcMotor.Direction.FORWARD);
//         
//         frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         
//         leftArm = hardwareMap.get(DcMotor.class, "motor3");
//         rightArm = hardwareMap.get(DcMotor.class, "motor4");
//         
//         leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         
//         leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         
// 
//         leftArm.setDirection(DcMotor.Direction.FORWARD);
//         rightArm.setDirection(DcMotor.Direction.REVERSE);
//         
//         leftArm.setTargetPosition(-35);
//         rightArm.setTargetPosition(-35);
//         
//         leftArm.setPower(.5);
//         rightArm.setPower(.5);
//         
//         leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//         rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//             servo2.setPosition(.1);  // Claw
//             servo3.setPosition(0.26);  // Wrist
//         
//       
//         double flpower, frpower, blpower, brpower;
//         double drive = 0;
//         double strafe = 0;
//         double turn = 0;
//         waitForStart();
// 
//     
// 
//         while (opModeIsActive()) {
//       
//             drive = gamepad1.left_stick_y ;
//             strafe =  gamepad1.left_stick_x; // D-Pad left/right for strafing ;
//             turn = gamepad1.right_stick_x;
//             
//             frpower = drive - turn + strafe;
//             brpower = drive - turn - strafe;
//             flpower = drive + turn - strafe;
//             blpower = drive + turn + strafe;
//             
//             double maxPower = Math.max(+- Math.abs(frpower),Math.max(Math.abs(brpower),Math.max(Math.abs(flpower), Math.abs(blpower))));
//             
//             if(maxPower > 1){
//                 frpower /= maxPower * .000000001;
//                 brpower /= maxPower * .000000001;
//                 flpower /= maxPower * .000000001;
//                 blpower /= maxPower * .000000001;
//             }
//             
//             frontleftDrive.setPower(flpower);
//             frontrightDrive.setPower(frpower);
//             backleftDrive.setPower(blpower);
//             backrightDrive.setPower(brpower);
//             
//              
//             
//             if (gamepad1.left_bumper && !leftBumperPressed) {
//                 // Activate slow mode
//                 speedMultiplier = SLOW_SPEED;
//                 slowModeActive = true;
//                 leftBumperPressed = true;
//             } else if (!gamepad1.left_bumper) {
//                 leftBumperPressed = false;
//             }
//             
//             if (gamepad1.right_bumper && !rightBumperPressed) {
//                 // Return to normal speed
//                 speedMultiplier = 1.0;
//                 slowModeActive = false;
//                 rightBumperPressed = true;
//             } else if (!gamepad1.right_bumper) {
//                 rightBumperPressed = false;
//             }
//             
//         
//             
//             
//             
//               if (gamepad2.left_bumper) {
//                 leftArm.setTargetPosition(-1680);
//                 rightArm.setTargetPosition(-1680);
//             
//                 leftArm.setPower(.5);
//                 rightArm.setPower(.5);
//             
//                 leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                 rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//             }
//             
//             if (gamepad2.right_bumper) {
//                 leftArm.setTargetPosition(-450);
//                 rightArm.setTargetPosition(-450);
//             
//                 leftArm.setPower(.4);
//                 rightArm.setPower(.4);
//             
//                 leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                 rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//             }
//             
// 
//     
//           
//             
//             
//             
//             if (gamepad1.dpad_up ||  gamepad2.dpad_up) {
//                 leftArm.setTargetPosition(-550);
//                 rightArm.setTargetPosition(-550);
//             
//                 leftArm.setPower(.5);
//                 rightArm.setPower(.5);
//             
//                 leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                 rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//             }
//            
//            
//             
//             
//             
//             
//             if (gamepad1.dpad_down|| gamepad2.dpad_down) {
//                 leftArm.setTargetPosition(-40);
//                 rightArm.setTargetPosition(-40);
//         
//                 leftArm.setPower(1);
//                 rightArm.setPower(1);
//         
//                 leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                 rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//             }
//             
//             if (gamepad1.dpad_right|| gamepad2.dpad_right) {
//                 leftArm.setTargetPosition(-1700);
//                 rightArm.setTargetPosition(-1700);
//                 servo3.setPosition(.45);
//                 
//                 leftArm.setPower(.5);
//                 rightArm.setPower(.5);
//         
//                 leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                 rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//             }
//             
//             
//             if (gamepad2.left_trigger > 0.05) {
//                 leftArm.setTargetPosition(leftArm.getCurrentPosition() + 60);
//                 rightArm.setTargetPosition(rightArm.getCurrentPosition() + 60);
//         
//                 leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                 rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//             }
//             
//             if (gamepad2.right_trigger > 0.05) {
//                 leftArm.setTargetPosition(leftArm.getCurrentPosition() - 60);
//                 rightArm.setTargetPosition(rightArm.getCurrentPosition() - 60);
//         
//                 leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                 rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//             }
//             
//             if(gamepad1.b || gamepad2.b) {
//                 servo3.setPosition(.6);
//             } if (gamepad1.y || gamepad2.y) {
//                 servo3.setPosition(.26);
//                 
//             } else if(gamepad1.x || gamepad2.x) {
//                 servo2.setPosition(.6);
//             } else if (gamepad1.a || gamepad2.a) {
//                 servo2.setPosition(.8);
//             } 
//             
//             telemetry.update();
//             
//             if (gamepad1.guide) {
//         telemetry.addData("Button Pressed", "Left Bumper");
//     }
//     if (gamepad1.start) {
//         telemetry.addData("Button Pressed", "Right Bumper");
//     }
//     
//         
//             
//             
//             telemetry.addData("Left Arm Power", leftArmPower);
//             telemetry.addData("Right Arm Power", rightArmPower);
//             telemetry.addData("Left Arm position", leftArm.getCurrentPosition());
//             telemetry.addData("Right Arm position", rightArm.getCurrentPosition());
//              telemetry.addData("Wrist position:", servo3.getPosition());
//              telemetry.addData("Claw position:", servo2.getPosition());
// 
//             telemetry.update();
//         }
//         
// 
//     
//           
//             
//            
// }
// }
