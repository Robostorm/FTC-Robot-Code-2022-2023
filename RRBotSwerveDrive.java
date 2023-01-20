package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controls the robot's swerve drive base
 * @author Visvam Rajesh
 * @since 2022-10-12
 */

public class RRBotSwerveDrive
{
    RRBotHardware robot;

    private ElapsedTime autoMoveTime = new ElapsedTime();

    private boolean isAutoMove = false;
    private double autoTime;

    /** Set the constants for the PID controller */
    //TODO: Set constants to their proper values
    private final double Kp = 0.02;
    private final double Ki = 0.01;
    private final double Kd = 0.001;
    private final PIDCoefficients pidCoef = new PIDCoefficients(Kp, Ki, Kd);

    private boolean hasNotReached; // Condition to escape PID Control Loop

    private int lastError; // The last error of the swerve module
    private int encoderPosition = 0; // current robot encoder position
    private int reference = 0; // goal of the swerve module

    /**
     * Constructor gets hardware object from teleop class
     * @param robot contains the hardware elements of the robot
     */
    public RRBotSwerveDrive(RRBotHardware robot)
    {
        this.robot = robot;
    }

    /**
     * Calculates the power of the motors for a swerve drive
     * @param leftX X position of the left joystick
     * @param leftY Y position of the left joystick
     */
    public double getPower(double leftX, double leftY)
    {
        //The drive power is the hypotenuse of the right triangle formed by leftX and leftY
        return Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
    }

//    /**
//     * Calculates the velocity values for the drive motors in a mecanum configuration.
//     * @param leftX X position of left joystick
//     * @param leftY Y position of left joystick
//     * @param rightX X position of right joystick
//     * @param rightY Y position of right joystick
//     * @return velocities - array of motor velocities
//     */
//    public double[] calcVelocities(double leftX, double leftY, double rightX, double rightY)
//    {
//        double moveX = rightX;
//        double moveY1 = leftY;
//        double turn = leftX;
//        double moveY2 = rightY;
//
//        double v1 = moveY1 + moveX + turn + moveY2;
//        double v2 = moveY1 - moveX - turn + moveY2;
//        double v3 = moveY1 + moveX - turn + moveY2;
//        double v4 = moveY1 - moveX + turn + moveY2;
//
//        double max = Math.abs(v1);
//        if(Math.abs(v2) > max)
//            max = Math.abs(v2);
//        if(Math.abs(v3) > max)
//            max = Math.abs(v3);
//        if(Math.abs(v4) > max)
//            max = Math.abs(v4);
//        if(max > 1)
//        {
//            v1 /= max;
//            v2 /= max;
//            v3 /= max;
//            v4 /= max;
//        }
//
//        double[] velocities = {v1, v2, v3, v4};
//        return velocities;
//    }

    /**
     * Sets the motor power for manual drive. The parameters are sent to calcVelocities.
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     */
    public void setMotorPower(double leftX, double leftY)
    {
        //calculate the velocities
        //double[] velocities = calcVelocities(leftX, leftY, rightX, rightY);
        double power = getPower(leftX, leftY);
        int angle = (int)getAngle(leftX, leftY);

        if(angle >= 180)
            power *= -1;

        //set the motor power

        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(power);
        robot.rearLeftDrive.setPower(power);
        robot.rearRightDrive.setPower(power);
    }

    /**
     * Calculates the angle(in degrees) of the servos in a standard swerve drive
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     */
    public double getAngle(double leftX, double leftY)
    {
        //The angle is the inverse tangent of the right triangle formed by leftX and leftY
        double radians = Math.atan(leftY / leftX);

        return (radians * 180) / Math.PI;
    }

    /**
     * Turns all servos based on the left stick position
     * @param leftX the x-axis of the left stick
     * @param leftY the y-axis of the left stick
     */
    public void setServoAngle(double leftX, double leftY)
    {
        double angle = getAngle(leftX, leftY); // Goal Angle

        // Use other setServoAngle method to control each servo
        setServoAngle(angle, RRBotHardware.SERVOS.FRONT_LEFT);
        setServoAngle(angle, RRBotHardware.SERVOS.FRONT_RIGHT);
        setServoAngle(angle, RRBotHardware.SERVOS.REAR_LEFT);
        setServoAngle(angle, RRBotHardware.SERVOS.REAR_RIGHT);
    }

    public void setServoAngle(double angle, RRBotHardware.SERVOS servo)
    {
        /**
         * Only needed if encoder increments infinitely *
        int startEncVal = 0;
        if(servo == RRBotHardware.SERVOS.FRONT_LEFT)
            startEncVal = (int)(robot.frontLeftEnc.getVoltage() * robot.ENCODER_TO_ANGLE);
        if(servo == RRBotHardware.SERVOS.FRONT_RIGHT)
            startEncVal = (int)(robot.frontRightEnc.getVoltage() * robot.ENCODER_TO_ANGLE);
        if(servo == RRBotHardware.SERVOS.REAR_LEFT)
            startEncVal = (int)(robot.rearLeftEnc.getVoltage() * robot.ENCODER_TO_ANGLE);
        if(servo == RRBotHardware.SERVOS.REAR_RIGHT)
            startEncVal = (int)(robot.rearRightEnc.getVoltage() * robot.ENCODER_TO_ANGLE);
         */

        reference = (int) (angle); // Goal Encoder Value

        if(reference >= 180)
            reference -= 180;

        // Check if reference is a negative angle, if so, change it to a positive one.
        if(reference < 0)
            reference += 360;

        if(servo == RRBotHardware.SERVOS.FRONT_LEFT)
            robot.frontLeftTurn.setPosition(angle); // swap fauxEncVal for robot.frontLeftEnc.getVoltage()
        if(servo == RRBotHardware.SERVOS.FRONT_RIGHT)
            robot.frontRightTurn.setPosition(angle); // swap fauxEncVal for robot.frontRightEnc.getVoltage()
        if(servo == RRBotHardware.SERVOS.REAR_LEFT)
            robot.rearLeftTurn.setPosition(angle); // swap fauxEncVal for robot.rearLeftEnc.getVoltage()
        if(servo == RRBotHardware.SERVOS.REAR_RIGHT)
            robot.rearRightTurn.setPosition(angle);

        /**The code below is old. Look at it if you want*/

//        ElapsedTime pidTimer = new ElapsedTime(); // Elapsed Time of each iteration of the PID Control Loop
//
//        hasNotReached = true; // Condition to escape PID Control Loop
//
//        // Values we will use through out the duration of the PID Control Loop
//        double integralSum = 0;
//        lastError = 0;
//
//        //TODO: Write code to set servo position based on an angle in degrees
//
//        //double fauxEncVal = Math.random() * 3.3;


//        while(hasNotReached)
//        {
//            // Get Current Encoder Position
//            encoderPosition = 0; // If it increments infinitely, then subtract startEncVal from encoderPosition
//            if(servo == RRBotHardware.SERVOS.FRONT_LEFT)
//                encoderPosition = (int)(robot.frontLeftEnc.getVoltage() * robot.ENCODER_TO_ANGLE); // swap fauxEncVal for robot.frontLeftEnc.getVoltage()
//            if(servo == RRBotHardware.SERVOS.FRONT_RIGHT)
//                encoderPosition = (int)(robot.frontRightEnc.getVoltage() * robot.ENCODER_TO_ANGLE); // swap fauxEncVal for robot.frontRightEnc.getVoltage()
//            if(servo == RRBotHardware.SERVOS.REAR_LEFT)
//                encoderPosition = (int)(robot.rearLeftEnc.getVoltage() * robot.ENCODER_TO_ANGLE); // swap fauxEncVal for robot.rearLeftEnc.getVoltage()
//            if(servo == RRBotHardware.SERVOS.REAR_RIGHT)
//                encoderPosition = (int)(robot.rearRightEnc.getVoltage() * robot.ENCODER_TO_ANGLE); // swap fauxEncVal for robot.rearRightEnc.getVoltage()
//
//            if(encoderPosition >= 355)
//                encoderPosition = 0;
//
//            // Calculate error
//            int error = reference - encoderPosition;
//
//            // Calculate Rate of Change of the error
//            double derivative = (error - lastError) / pidTimer.seconds();
//
//            // Calculate Sum of All error over time
//            integralSum = integralSum + (error * pidTimer.seconds());
//
//            // Set servo power
//            double out = (pidCoef.p * error) + (pidCoef.i * integralSum) + (pidCoef.d * derivative);
//
//            /**  Note to self: this loop will most likely have to be changed to account for the different values of the individual swerve modules*/
//
//            if(servo == RRBotHardware.SERVOS.FRONT_LEFT)
//                robot.frontLeftTurn.setPower(out);
//            if(servo == RRBotHardware.SERVOS.FRONT_RIGHT)
//                robot.frontRightTurn.setPower(out);
//            if(servo == RRBotHardware.SERVOS.REAR_LEFT)
//                robot.rearLeftTurn.setPower(out);
//            if(servo == RRBotHardware.SERVOS.REAR_RIGHT)
//                robot.rearRightTurn.setPower(out);
//
//            lastError = error;
//
//            if(lastError <= 15)
//                hasNotReached = false;
//        }
//        if(servo == RRBotHardware.SERVOS.FRONT_LEFT)
//            robot.frontLeftTurn.setPower(0);
//        if(servo == RRBotHardware.SERVOS.FRONT_RIGHT)
//            robot.frontRightTurn.setPower(0);
//        if(servo == RRBotHardware.SERVOS.REAR_LEFT)
//            robot.rearLeftTurn.setPower(0);
//        if(servo == RRBotHardware.SERVOS.REAR_RIGHT)
//            robot.rearRightTurn.setPower(0);
    }

    /**
     * Turns the robot to face a specific direction based on right stick position. Meant to be used in teleop
     * @param rightX the x-axis of the right stick
     * @param rightY the y-axis of the right stick
     */
    public void TurnFacing(double rightX, double rightY)
    {
        // Get Angle to Turn to
        double angle = getAngle(rightX, rightY);

        // Turn each servo individually
        if(angle > 10)
        {
            setServoAngle(angle, RRBotHardware.SERVOS.FRONT_LEFT);
            setServoAngle(90.0 + angle, RRBotHardware.SERVOS.FRONT_RIGHT);
            setServoAngle(180.0 + angle, RRBotHardware.SERVOS.REAR_RIGHT);
            setServoAngle(270.0 + angle, RRBotHardware.SERVOS.REAR_LEFT);
        }
    }
    /**
     * Automatically moves the robot based on a set speed and time. Meant to be used in teleop.
     * @param speed speed of movement
     * @param time how long the movement should take
     */
    public void AutoMove(double speed, double time)
    {
        isAutoMove = true;
        autoTime = time;

        autoMoveTime.reset();

        robot.rearRightDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
    }

    /**
     * Checks if the movement is done by comparing the time elapsed and the time the movement is set to take.
     */
    public void AutoMoveEndCheck()
    {
        if(autoMoveTime.milliseconds() >= autoTime)
        {
            isAutoMove = false;
        }
    }

    /**
     * Returns the last error of the swerve module
     * @return lastError
     */
    public int getLastError(){ return lastError; }

    /**
     * Returns the goal encoder position of the swerve module
     * @return reference
     */
    public int getReference(){ return reference; }

    /**
     * Returns the current encoder position of the swerve module
     * @return encoderPosition
     */
    public int getEncoderPosition(){ return encoderPosition; }

    /**
     * Returns if the swerve module has reached its goal
     * @return hasNotReached
     */
    public boolean getHasNotReached(){ return hasNotReached; }

    /**
     * Returns whether an auto move is currently occurring
     * @return isAutoMove
     */
    public boolean getIsAutoMove()
    {
        return isAutoMove;
    }
}