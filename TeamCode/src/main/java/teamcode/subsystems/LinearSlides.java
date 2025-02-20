/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import teamcode.Robot;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcPidController;

/**
 * This class creates the LinearSlides subsystem of the Extender Arm.
 */
public class LinearSlides
{
    /**
     * This class contains LinearSlides subsystem constants and parameters.
     */
    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "LinearSlides";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final FtcMotorActuator.MotorType PRIMARY_MOTOR_TYPE = FtcMotorActuator.MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;

        public static final double ENCODER_CPR                  = 28;
        public static final double GEAR_RATIO                   = 143.06;
        public static final double DEG_SCALE                    = 360.0 / (ENCODER_CPR * GEAR_RATIO);
        public static final double POS_OFFSET                   = -130.86;
        public static final double ZERO_OFFSET                  = 0.0;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.2;

        public static final double PIVOT_OFFSET                 = 4.202;    // Measured from CAD in inches
        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 105.0;
        public static final double START_POS                    = MIN_POS;
        public static final double RESTRICTED_POS_THRESHOLD     = 40.0;
        public static final double GROUND_PICKUP_POS            = 20.0;
        public static final double SPECIMEN_PICKUP_POS          = 17.85;
        public static final double LOW_BASKET_SCORE_POS         = 105.0;
        public static final double HIGH_BASKET_SCORE_POS        = 104.5;
        public static final double LOW_CHAMBER_SCORE_POS        = 40.0;
        public static final double HIGH_CHAMBER_SCORE_POS       = 95.0;
        public static final double PRE_CLIMB_POS                = 55.0;
        public static final double ASCENT_LEVEL1_POS            = 45.0;
        public static final double LEVEL2_RETRACT_POS           = 21.0;
        public static final double LEVEL2_TORQUE_POS            = 47.0;
        public static final double LEVEL2_FINAL_POS             = 2.0;
        public static final double[] posPresets                 = {MIN_POS, 30.0, 60.0, 90.0, MAX_POS};
        public static final double POS_PRESET_TOLERANCE         = 10.0;

        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.125, 0.0, 0.002, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 3.0;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.2/10;
    }   //class Params

    private final Robot robot;
    public final TrcMotor linearSlides;
    private TrcPidController linearSlidesPID;

    /**
     * Constructor: Creates an instance of the object.
     */
    public LinearSlides(Robot robot)
    {
        this.robot = robot;
        FtcMotorActuator.Params LinearSlidesParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_SCALE, Params.POS_OFFSET, Params.ZERO_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        linearSlides = new FtcMotorActuator(LinearSlidesParams).getMotor();
        linearSlides.setSoftwarePidEnabled(true);
        linearSlides.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE);
        linearSlides.setPositionPidPowerComp(this::getLinearSlidesPowerComp);
        linearSlidesPID = linearSlides.getPositionPidController();
        linearSlidesPID.setNoOscillation(true);
        linearSlides.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
    }   //LinearSlides

    /**
     * This method returns the created LinearSlides motor.
     *
     * @return created LinearSlides motor.
     */
    public TrcMotor getMotor()
    {
        return linearSlides;
    }   //getMotor

    /**
     * This method is called to compute the power compensation to counteract gravity on the LinearSlides.
     *
     * @param currPower specifies the current motor power (not used).
     * @return gravity compensation for the arm.
     */
    private double getLinearSlidesPowerComp(double currPower)
    {
        /*if (robot.extender != null)
        {
            double extenderLength = robot.extender.getPosition();
            double extenderAngleRadian =
                Math.toRadians(LinearSlides.getPosition()) - Math.atan(Params.PIVOT_OFFSET/extenderLength);
            // Calculate extender floor distance from the pivot point.
            double extenderFloorDistanceFromPivot =
                TrcUtil.magnitude(Params.PIVOT_OFFSET, extenderLength) * Math.cos(extenderAngleRadian);
            // Extender angle is zero horizontal.
            return Params.GRAVITY_COMP_MAX_POWER * extenderFloorDistanceFromPivot;
        }
        else
        {
            // We should not be calling gravity comp if extender has not been created.
            // But to avoid NullPointerException just in case, return 0.0 gravity comp power.
            return 0.0;
        }*/
        return 0.0;
    }   //getLinearSlidesPowerComp

}   //class LinearSlides
