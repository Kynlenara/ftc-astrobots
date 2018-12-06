/*
Copyright 2018 FIRST Tech Challenge Team 14726

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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous(name="CraterAutonomous 1.4.5 (Recommended)", group="Linear Opmode")

public class StableAutonomous2 extends LinearOpMode {
    //MOTORS
    //Left wheel motor
    private DcMotor leftDrive = null;
    //Right wheel motor
    private DcMotor rightDrive = null;
    //Hook arm motor
    private DcMotor armDrive = null;
    //Cup arm motor
    private DcMotor cupDrive = null;
    //SERVOS
    //Cup arm servo
    public Servo armServo = null;
    //Servo for rotating the cup
    public Servo cupServo = null;
    //Hook servo
    public Servo hookServo = null;
    
    private ElapsedTime runtime = new ElapsedTime();
    
    //Function that waits a specified number of seconds and provides telemetry
    public void waitSeconds(double S){
        double endTime = runtime.seconds()+S;
        while(runtime.seconds()<endTime && opModeIsActive()){
            double remaining = endTime-runtime.seconds();
            telemetry.addLine("Robot is waiting...");
            telemetry.addData("Time remaining", remaining + " seconds");
            telemetry.update();
        }
    }
    //Function for setting all servos
    public void servoSet(double A, double C, double H){
        armServo.setPosition(A);
        cupServo.setPosition(C);
        hookServo.setPosition(H);
    }
    //Function for driving forward a specified amount of centimeters
    public void driveCm(int Dist, double Power,double Adjust){
        /*easeIn is a condition that allows the wheel motors to reduce their power
        as the robot gets closer to the target distance.
        */
        boolean easeIn = false;
        //easeIn is activated if the provided Power is less than 0
        if(Power<0){
            easeIn = true;
            Power = 1;
        }
        //complete indicates if the robot has reached the target distance yet
        boolean complete = false;
        double timeout = -1;
        //Current position of both wheel motors during initialization of the function
        int prevL = leftDrive.getCurrentPosition();
        int prevR = rightDrive.getCurrentPosition();
        //Setting power
        leftDrive.setPower(Power);
        rightDrive.setPower(Power);
        /*Setting the target position to the current position plus the
        target distance in centimeters
        */
        leftDrive.setTargetPosition(prevL+Dist*10);
        rightDrive.setTargetPosition(prevR+Dist*10);
        //Loop to hold the program until motors have completed their drive
        while(leftDrive.isBusy() || rightDrive.isBusy() && opModeIsActive()) {
            //Variable of the amount of clicks driven during the function
            int driven = ((leftDrive.getCurrentPosition()+rightDrive.getCurrentPosition())/2)-((prevL+prevR)/2);
            //Amount of distance left until the robot reaches the target distance
            double remaining = Dist-(driven/10);
            //easeIn sets wheel motor power here.
            if(easeIn){
                double easeMult = remaining/Dist;
                leftDrive.setPower(0.25+Math.abs(easeMult*0.75));
                rightDrive.setPower(0.25+Math.abs(easeMult*0.75));
            }
            //--------------Telemetry---------------
            telemetry.addLine("Wheel motors are running.");
            //Target distance in centimeters
            telemetry.addData("Target Distance", Dist + "cm");
            //Current distance in centimeters
            telemetry.addData("Current Distance", driven/10 + "cm");
            //Average power of both motors (generally the same...)
            telemetry.addData("Power", (leftDrive.getPower()+rightDrive.getPower())/2);
            //Remaining distance in centimeters
            telemetry.addData("Remaining", remaining);
            telemetry.update();
            /*Marks drive as "complete" and marks current elapsed time from the
            start of the opMode.
            */
            if(remaining<1 && !complete){
                complete = true;
                timeout = runtime.seconds();
            }
            /*If the difference of the current elapsed time and the time of
            completion exceeds the parameter "Adjust", the function will end.
            This allows you to give precise movements more adjustment time in
            case the motors overshoot.
            */
            if(runtime.seconds()-timeout>Adjust && complete){return;}
        }
    }
    
    //Function for turning the robot a certain amount of degrees by using the wheels
    public void turnDeg(int Deg,double Power,double Adjust){
        //Identical to the usage in driveCm
        boolean easeIn = false;
        if(Power<0){
            easeIn = true;
            Power = 1;
        }
        boolean complete = false;
        double timeout = -1;
        //Multiplier of degrees to clicks (1 degree = 2.7 clicks)
        double degMult = 2.7;
        //Current position of both wheel motors during initialization of the function
        int prevL = leftDrive.getCurrentPosition();
        int prevR = rightDrive.getCurrentPosition();
        //Setting power
        leftDrive.setPower(Power);
        rightDrive.setPower(Power);
        /*Amount of degrees that were inputted are converted to the amount of
        clicks and saves it as a double.
        */
        Double degDouble = Deg*degMult;
        //Same thing as above but converted to an integer
        int targ = degDouble.intValue();
        //Sets target position to current position plus calculated target position
        leftDrive.setTargetPosition(prevL+targ);
        rightDrive.setTargetPosition(prevR-targ);
        //Loop to hold the program until motors have completed their turn
        while(leftDrive.isBusy() || rightDrive.isBusy() && opModeIsActive()) {
            //Amount of degrees turned
            int driven = ((leftDrive.getCurrentPosition()-rightDrive.getCurrentPosition())/2)-((prevL-prevR)/2);
            //Amount of degrees left in the turn
            double remaining = Deg-(driven/degMult);
            //easeIn sets wheel motor power here.
            if(easeIn){
                double easeMult = remaining/Deg;
                leftDrive.setPower(0.25+Math.abs(easeMult*0.75));
                rightDrive.setPower(0.25+Math.abs(easeMult*0.75));
            }
            telemetry.addLine("Wheel motors are running.");
            //Target angle in degrees
            telemetry.addData("Target Angle", Deg + "°");
            //Current angle in degrees
            telemetry.addData("Current Angle", driven/degMult + "°");
            //Degrees remaining before turn is complete
            telemetry.addData("Degrees Remaining", remaining + "°");
            //Average power of both wheel motors (they will probably be the same...)
            telemetry.addData("Power", (leftDrive.getPower()+rightDrive.getPower())/2);
            telemetry.update();
            //Identical timeout stuff from function driveCm
            if(remaining<1 && !complete){
                complete = true;
                timeout = runtime.seconds();
            }
            if(runtime.seconds()-timeout>Adjust && complete){return;}
        }
    }
    //Sets the position of the cup arm motor.
    public void cupSet(int Clicks,double Power){
        cupDrive.setPower(Power);
        cupDrive.setTargetPosition(Clicks);
    }
    //Sets the position of the hook arm motor.
    public void eleSet(int Clicks,double Power){
        armDrive.setPower(Power);
        armDrive.setTargetPosition(Clicks);
        //Loop to hold the program until the motor has turned enough.
        while(armDrive.isBusy() && opModeIsActive()) {
            //Telemetry is probably self-explanatory
            telemetry.addLine("Hook arm motor is running.");
            telemetry.addData("Target Position", Clicks + " clicks");
            telemetry.addData("Current Position", armDrive.getCurrentPosition() + " clicks");
            telemetry.addData("Power", armDrive.getPower());
            telemetry.update();
            if(Clicks-armDrive.getCurrentPosition()<1){return;}
        }
    }
    
    @Override
    public void runOpMode() {
        //Mapping
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDcMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDcMotor");
        armDrive = hardwareMap.get(DcMotor.class, "eleArmMotor");
        cupDrive = hardwareMap.get(DcMotor.class, "cupArmMotor");
        armServo = hardwareMap.get(Servo.class, "servo2");
        cupServo = hardwareMap.get(Servo.class, "servo3");
        hookServo = hardwareMap.get(Servo.class, "servo1");
        
        //Setting directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.REVERSE);
        cupDrive.setDirection(DcMotor.Direction.FORWARD);
        
        //Setting proper modes
        //Resetting positions first
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cupDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cupDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Activating brakes for zero power
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cupDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //Initializes Servos
        servoSet(1,0,1);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //---------------ADD ALL DRIVE FUNCTIONS HERE---------------

            //Move hook arm all the way up to lower robot
            eleSet(20000,1);
            //Move hook servo down and wait
            servoSet(1,0,0.75);
            waitSeconds(1);
            //Drive forward 80 cm
            driveCm(55,1,1);
            //Turn 90 degrees right
            turnDeg(80,1,1.5);
            //Drive forward 105 cm
            driveCm(80,1,2);
            //Turn 45 degrees right
            turnDeg(45,1,1);
            driveCm(90,1,1);
            //Deploy arm to drop team marker
            servoSet(0,1,0.75);
            cupSet(600,1);
            waitSeconds(3);
            //Retract arm
            servoSet(1,0,0.75);
            waitSeconds(3);
            //-----------------END OF DRIVE FUNCTIONS-------------------
            //Not sure if this is necessary
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //Breaks loop (ending opMode)
            break;
        }
    }
}
