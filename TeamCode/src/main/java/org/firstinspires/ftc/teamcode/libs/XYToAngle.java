package org.firstinspires.ftc.teamcode.libs;

//import static android.icu.util.MeasureUnit.DEGREE;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class XYToAngle {
    /*** Processes input xy controller input to an output angles and power
     * Default parameters
     * should scale to circle boolean = Yes
     *      This will make sure that the power never exceeds 1
     *      even though the controller will return up to 1.2 in the corners
     * should scale for speed boolean = Yes
     *      This will make sure that the speed ramps up slowly so that the robot has more sensitivity
     *      This will have a counter effect on size of dead spot - effectively spreading the dead size
     * Type of angle in AngleUnit = DEGREES
     *      Alternative is RADIANS
     * size of dead spot = 0.0;
     *      Most controllers have a dead spot in the middle of the stick
     *      this parameter allows you to make it bigger (in a circle)
     */
    double x = 0;
    double y = 0;
    Double power = 0.0;
    Double angle = 0.0;

    boolean scaleToCircle = false;
    boolean scaleForSpeed = true;
    AngleUnit type = AngleUnit.DEGREES;
    Double deadSize = 0.0;

    /***
     * init class with defaults
     */
    public XYToAngle(){ }

    /***
     * init and set shouldCircleScale (boolean) and shouldSlowScale (boolean)
     * @param shouldCircleScale
     * @param shouldSlowScale
     */
    public XYToAngle(boolean shouldCircleScale, boolean shouldSlowScale) {
        scaleToCircle = shouldCircleScale;
        scaleForSpeed = shouldSlowScale;
    }

    /***
     * init with type angle only - the only real choice here is AngleUnit.RADIANS
     * @param typeAngle
     * @throws Exception
     */
    public XYToAngle(AngleUnit typeAngle) throws Exception {
        type = typeAngle;
    }

    /***
     * init with defined size of dead spot
     *  NOTE: anything 1 or larger will kill the controller output
     * @param sizeOfDeadSpot
     */
    public XYToAngle(Double sizeOfDeadSpot){
        deadSize = sizeOfDeadSpot;
    }
    /***
     * init with all three custom settings
     * @param shouldCircleScale
     * @param shouldSlowScale
     * @param typeAngle
     * @param sizeOfDeadSpot
     * @throws Exception
     */
    public XYToAngle(
            boolean shouldCircleScale,
            boolean shouldSlowScale,
            AngleUnit typeAngle,
            Double sizeOfDeadSpot) throws Exception {
        type = typeAngle;
        scaleToCircle = shouldCircleScale;
        scaleForSpeed = shouldSlowScale;
        deadSize = sizeOfDeadSpot;
    }

    /***
     * setXY
     *  takes in the x controller and y controller in preparation for getting out either the angle
     *  or the power or both
     * @param x_stick
     * @param y_stick
     */
    public void setXY(double x_stick, double y_stick){
        x = scaleInput(x_stick);
        y = scaleInput(y_stick);
//        opMode.telemetry.addData("controller 1 left stick", "X: (%.2f), Y: (%.2f)", x, y);
        Double first_angle = Math.atan2(y, x);
        Double first_power = sqrt(Math.pow(x, 2) + Math.pow(y,2));
        Double fullPower = sqrt(Math.pow(Math.tan(first_angle), 2) + 1);
        if (type.equals(AngleUnit.DEGREES)) {
            first_angle = Math.toDegrees(first_angle);
        }
        if(scaleToCircle){
            first_power = first_power / fullPower;
        }
        if (first_power > 1){
            first_power = 1.0;
        }
        angle = first_angle;
        power = first_power;
//        opMode.telemetry.addData("direction and power", "angle: (%.2f), power: (%.2f)", angle, power);
    }

    /***
     * getting the angle based on curently set x and y
     * @return the angle
     */
    public double getAngle(){
        return angle;
    }

    /***
     * returns the power based on currently set x and y
     * @return
     */
    public double getPower(){
        return power;
    }

    /***
     * scale the input down (if scaleForSpeed) set so that the stick is more sensitive
     * @param input
     * @return
     */
    private Double scaleInput(Double input){
        Double output = input;
        if (scaleForSpeed){
            int input_sense = 1;
            if (input < 0){
                input_sense = -1;
            }
            output = input * input * input_sense;
        }
        return output;
    }
}
