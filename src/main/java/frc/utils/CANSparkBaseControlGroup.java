package frc.utils;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import java.util.HashMap;

public class CANSparkBaseControlGroup {
    private final HashMap<Integer, CANSparkBase> deviceIDToSparkBase = new HashMap<>();

    /**
     * Constructor for a group of CANSparkMax motors.
     * 
     * @param sparkBases The motors to be included in the control group.
     * @return A utility collection of CANSparkBase objects.
     */
    public CANSparkBaseControlGroup(CANSparkBase... sparkBases) {
        for (CANSparkBase sparkBase : sparkBases) {
            deviceIDToSparkBase.put(sparkBase.getDeviceId(), sparkBase);
        }
    }

    /**
     * Constructor for a group of CANSparkMax motors.
     * 
     * @param areBrushless true if the motors are brushless, false if brushed.
     * @param deviceIDs    the device IDs of the motors.
     * @return A utility collection of CANSparkBase objects.
     */
    public CANSparkBaseControlGroup(boolean areBrushless, int... deviceIDs) {
        for (int deviceID : deviceIDs) {
            deviceIDToSparkBase.put(deviceID, new CANSparkMax(deviceID, (areBrushless ? MotorType.kBrushless : MotorType.kBrushed)));
        }
    }

    /**
     * Gets the number of objects within the control group.
     * 
     * @return The number of objects within the control group.
     */
    public int getSize() {
        return deviceIDToSparkBase.size();
    }

    /**
     * Gets the list of CANSparkBase objects within the control group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The list of CANSparkBase objects within the control group.
     */
    public CANSparkBase getSparkBaseByID(int deviceID) {
        return deviceIDToSparkBase.get(deviceID);
    }

    /**
     * Sets the speed of the motors in the group.
     * 
     * @param speed The speed to set the control group to.
     */
    public void setSpeed(double speed) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.set(speed));
    }

    /**
     * Sets the speed of the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @param speed    The speed to set the motor to.
     */
    public void setSpeedByID(int deviceID, double speed) {
        getSparkBaseByID(deviceID).set(speed);
    }

    /**
     * Gets the speed of the motors in the group.
     * 
     * @return The speed of the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.
     * @throws UnsupportedOperationException If the value of one or more specific CANSparkBase objects was changed.
     */
    public double getSpeed() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        double firstSpeed = iterator.next().get();
        while (iterator.hasNext()) {
            if (iterator.next().get() != firstSpeed) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getAverageSpeed method of this class.");
            }
        }

        return firstSpeed;
    }

    /**
     * Gets the average speed of the motors in the group.
     * 
     * @return The average speed of the motors in the group.
     * @throws UnsupportedOperationException If the value of one or more specific CANSparkBase objects was changed.
     */
    public double getAverageSpeed() {
        try {
            this.getSpeed();
        } catch (UnsupportedOperationException e) {
            double sum = 0;
            for (CANSparkBase sparkBase : deviceIDToSparkBase.values()) {
                sum += sparkBase.get();
            }
            return sum / deviceIDToSparkBase.values().size();
        }
    }

    /**
     * Sets the idle mode of the motors in the group.
     * 
     * @param idleMode The idle mode to set the motors to.
     */
    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.setIdleMode(idleMode));
    }

    /**
     * Sets the idle mode of the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @param idleMode The idle mode to set the motor to.
     */
    public void setIdleModeByID(int deviceID, CANSparkMax.IdleMode idleMode) {
        getSparkBaseByID(deviceID).setIdleMode(idleMode);
    }

    /**
     * Gets the idle mode of the motors in the group.
     * 
     * @return The idle mode of the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.
     * @throws UnsupportedOperationException If the value of one or more specific CANSparkBase objects was changed.
     */
    public CANSparkMax.IdleMode getIdleMode() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        CANSparkMax.IdleMode firstIdleMode = iterator.next().getIdleMode();
        while (iterator.hasNext()) {
            if (iterator.next().getIdleMode() != firstIdleMode) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getIdleMode method of this class.");
            }
        }

        return firstIdleMode;
    }

    /**
     * Gets the idle mode of the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The idle mode of the motor.
     */
    public CANSparkMax.IdleMode getIdleModeByID(int deviceID) {
        return getSparkBaseByID(deviceID).getIdleMode();
    }

    /**
     * Sets the inverted state of the motors in the group.
     * 
     * @param isInverted true to invert the motors, false otherwise.
     */
    public void setInverted(boolean isInverted) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.setInverted(isInverted));
    }

    /**
     * Sets the inverted state of the motor in the group based on the device ID.
     * 
     * @param deviceID   The device ID of the motor.
     * @param isInverted true to invert the motor, false otherwise.
     */
    public void setInvertedByID(int deviceID, boolean isInverted) {
        getSparkBaseByID(deviceID).setInverted(isInverted);
    }

    /**
     * Gets the inverted state of the motors in the group.
     * 
     * @return The inverted state of the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.
     * @throws UnsupportedOperationException If the value of one or more specific CANSparkBase objects was changed.
     */
    public boolean getInverted() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        boolean firstInverted = iterator.next().getInverted();
        while (iterator.hasNext()) {
            if (iterator.next().getInverted() != firstInverted) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getInverted method of this class.");
            }
        }

        return firstInverted;
    }

    /**
     * Gets the inverted state of the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The inverted state of the motor.
     */
    public boolean getInvertedByID(int deviceID) {
        return getSparkBaseByID(deviceID).getInverted();
    }

    /**
     * Sets the smart current limit for the motors in the group.
     * 
     * @param currentLimit The smart current limit to set for the motors.
     */
    public void setSmartCurrentLimit(int currentLimit) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.setSmartCurrentLimit(currentLimit));
    }

    /**
     * Sets the smart current limit for the motor in the group based on the device ID.
     * 
     * @param deviceID     The device ID of the motor.
     * @param currentLimit The smart current limit to set for the motor.
     */
    public void setSmartCurrentLimitByID(int deviceID, int currentLimit) {
        getSparkBaseByID(deviceID).setSmartCurrentLimit(currentLimit);
    }

    /**
     * Gets the smart current limit for the motors in the group.
     * 
     * @return The smart current limit for the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.
     * @throws UnsupportedOperationException If the value of one or more specific CANSparkBase objects was changed.
     */
    public int getSmartCurrentLimit() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        int firstCurrentLimit = iterator.next().getSmartCurrentLimit();
        while (iterator.hasNext()) {
            if (iterator.next().getSmartCurrentLimit() != firstCurrentLimit) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getSmartCurrentLimit method of this class.");
            }
        }

        return firstCurrentLimit;
    }

    /**
     * Gets the smart current limit for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The smart current limit for the motor.
     */
    public int getSmartCurrentLimitByID(int deviceID) {
        return getSparkBaseByID(deviceID).getSmartCurrentLimit();
    }

    /**
     * Gets the average smart current limit for the motors in the group.
     * 
     * @return The average smart current limit for the motors in the group.
     */
    public int getAverageSmartCurrentLimit() {
        try {
            this.getSmartCurrentLimit();
        } catch (UnsupportedOperationException e) {
            int sum = 0;
            for (CANSparkBase sparkBase : deviceIDToSparkBase.values()) {
                sum += sparkBase.getSmartCurrentLimit();
            }
            return sum / deviceIDToSparkBase.values().size();
        }
    }

    /**
     * Sets the secondary current limit for the motors in the group.
     * 
     * @param currentLimit The secondary current limit to set for the motors.
     */
    public void setSecondaryCurrentLimit(double currentLimit) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.setSecondaryCurrentLimit(currentLimit));
    }
    
    /**
     * Sets the secondary current limit for the motor in the group based on the device ID.
     * 
     * @param deviceID     The device ID of the motor.
     * @param currentLimit The secondary current limit to set for the motor.
     */
    public void setSecondaryCurrentLimitByID(int deviceID, double currentLimit) {
        getSparkBaseByID(deviceID).setSecondaryCurrentLimit(currentLimit);
    }
    
    /**
     * Gets the secondary current limit for the motors in the group.
     * 
     * @return The secondary current limit for the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.
     * @throws UnsupportedOperationException If the value of one or more specific CANSparkBase objects was changed.
     */
    public double getSecondaryCurrentLimit() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        double firstCurrentLimit = iterator.next().getSecondaryCurrentLimit();
        while (iterator.hasNext()) {
            if (iterator.next().getSecondaryCurrentLimit() != firstCurrentLimit) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getSecondaryCurrentLimit method of this class.");
            }
        }

        return firstCurrentLimit;
    }

    /**
     * Gets the average secondary current limit for the motors in the group.
     * 
     * @return The average secondary current limit for the motors in the group.
     */    
    public double getAverageSecondaryCurrentLimit() {
        try {
            this.getSecondaryCurrentLimit();
        } catch (UnsupportedOperationException e) {
            double sum = 0;
            for (CANSparkBase sparkBase : deviceIDToSparkBase.values()) {
                sum += sparkBase.getSecondaryCurrentLimit();
            }
            return sum / deviceIDToSparkBase.values().size();
        }
    }

    /**
     * Gets the secondary current limit for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The secondary current limit for the motor.
     */
    public double getSecondaryCurrentLimitByID(int deviceID) {
        return getSparkBaseByID(deviceID).getSecondaryCurrentLimit();
    }


    /**
     * Sets the ramp rate for the motors in the group.
     * 
     * @param rampRate The ramp rate to set for the motors.
     */

    public void setRampRate(double rampRate) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.setOpenLoopRampRate(rampRate));
    }

    /**
     * Sets the ramp rate for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @param rampRate The ramp rate to set for the motor.
     */
    public void setRampRateByID(int deviceID, double rampRate) {
        getSparkBaseByID(deviceID).setOpenLoopRampRate(rampRate);
    }

    /**
     * Gets the ramp rate for the motors in the group.
     * 
     * @return The ramp rate for the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.

     */
    public double getRampRate() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        double firstRampRate = iterator.next().getOpenLoopRampRate();
        while (iterator.hasNext()) {
            if (iterator.next().getOpenLoopRampRate() != firstRampRate) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getRampRate method of this class.");
            }
        }

        return firstRampRate;
    }


    /**
    * Gets the average ramp rate for the motors in the group.
    * 
    * @return The average ramp rate for the motors in the group.
    */
    public double getAverageRampRate() {
        try {
            this.getRampRate();
        } catch (UnsupportedOperationException e) {
            double sum = 0;
            for (CANSparkBase sparkBase : deviceIDToSparkBase.values()) {
                sum += sparkBase.getOpenLoopRampRate();
            }
            return sum / deviceIDToSparkBase.values().size();
        }
    }


    /**
     * Gets the ramp rate for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The ramp rate for the motor.
     */

    public double getRampRateByID(int deviceID) {
        return getSparkBaseByID(deviceID).getOpenLoopRampRate();
    }

    /**
     * Sets the output range for the motors in the group.
     * 
     * @param minOutput The minimum output value.
     * @param maxOutput The maximum output value.
     */

    public void setOutputRange(double minOutput, double maxOutput) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.setClosedLoopRampRate(minOutput, maxOutput));
    }

    /**
     * Sets the output range for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @param minOutput The minimum output value.
     * @param maxOutput The maximum output value.
     */

    public void setOutputRangeByID(int deviceID, double minOutput, double maxOutput) {
        getSparkBaseByID(deviceID).setClosedLoopRampRate(minOutput, maxOutput);
    }


    /**
     * Gets the output range for the motors in the group.
     * 
     * @return The output range for the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.

     */

    public double[] getOutputRangeArray() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        double firstMinOutput = iterator.next().getClosedLoopRampRate();
        double firstMaxOutput = iterator.next().getClosedLoopRampRate();
        while (iterator.hasNext()) {
            if (iterator.next().getClosedLoopRampRate() != firstMinOutput) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getOutputRange method of this class.");
            }
        }

        return new double[] {firstMinOutput, firstMaxOutput};
    }

    public double getOutputRange(){
        double[] outputRange = getOutputRangeArray();
        return outputRange[1]-outputRange[0];
    }


    /**
     * Gets the average output range for the motor in the group based on the device ID
     * @return The average output range for the motor in the group based on the device ID
     */
    public double[] getAverageOutputRangeArray() {
        try {
            this.getOutputRange();
        } catch (UnsupportedOperationException e) {
            double sumMinOutput = 0;
            double sumMaxOutput = 0;
            for (CANSparkBase sparkBase : deviceIDToSparkBase.values()) {
                sumMinOutput += sparkBase.getClosedLoopRampRate();
                sumMaxOutput += sparkBase.getClosedLoopRampRate();
            }
            return new double[] {sumMinOutput / deviceIDToSparkBase.values().size(), sumMaxOutput / deviceIDToSparkBase.values().size()};
        }
    }


    /**
     * Gets the output range for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The output range for the motor.
     */

    public double[] getOutputRangeArrayByID(int deviceID) {
        return new double[] {getSparkBaseByID(deviceID).getClosedLoopRampRate(), getSparkBaseByID(deviceID).getClosedLoopRampRate()};
    }

    public double getOutputRangeByID(int deviceID){
        double[] outputRange = getOutputRangeArrayByID(deviceID);
        return outputRange[1]-outputRange[0];
    }

    /**
     * Sets the PIDF values for the motors in the group.
     * 
     * @param kP The proportional gain.
     * @param kI The integral gain.
     * @param kD The derivative gain.
     * @param kF The feedforward gain.
     */

    public void setPIDF(double kP, double kI, double kD, double kF) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.getPIDController().setP(kP));
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.getPIDController().setI(kI));
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.getPIDController().setD(kD));
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.getPIDController().setFF(kF));
    }

    /**
     * Sets the PIDF values for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @param kP The proportional gain.
     * @param kI The integral gain.
     * @param kD The derivative gain.
     * @param kF The feedforward gain.
     */

    public void setPIDFByID(int deviceID, double kP, double kI, double kD, double kF) {
        getSparkBaseByID(deviceID).getPIDController().setP(kP);
        getSparkBaseByID(deviceID).getPIDController().setI(kI);
        getSparkBaseByID(deviceID).getPIDController().setD(kD);
        getSparkBaseByID(deviceID).getPIDController().setFF(kF);
    }

    /**
     * Gets the PIDF values for the motors in the group.
     * 
     * @return The PIDF values for the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.
     * @throws UnsupportedOperationException If the value of one or more specific CANSparkBase objects was changed.
     */

    public double[] getPIDFArray() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        double firstkP = iterator.next().getPIDController().getP();
        double firstkI = iterator.next().getPIDController().getI();
        double firstkD = iterator.next().getPIDController().getD();
        double firstkF = iterator.next().getPIDController().getFF();
        while (iterator.hasNext()) {
            if (iterator.next().getPIDController().getP() != firstkP) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getPIDF method of this class.");
            }
        }

        return new double[] {firstkP, firstkI, firstkD, firstkF};
    }

    /**
     * Gets the average PIDF values for the motors in the group.
     * 
     * @return The average PIDF values for the motors in the group.
     */

    public double[] getAveragePIDFArray() {
        try {
            this.getPIDFArray();
        } catch (UnsupportedOperationException e) {
            double sumkP = 0;
            double sumkI = 0;
            double sumkD = 0;
            double sumkF = 0;
            for (CANSparkBase sparkBase : deviceIDToSparkBase.values()) {
                sumkP += sparkBase.getPIDController().getP();
                sumkI += sparkBase.getPIDController().getI();
                sumkD += sparkBase.getPIDController().getD();
                sumkF += sparkBase.getPIDController().getFF();
            }
            return new double[] {sumkP / deviceIDToSparkBase.values().size(), sumkI / deviceIDToSparkBase.values().size(), sumkD / deviceIDToSparkBase.values().size(), sumkF / deviceIDToSparkBase.values().size()};
        }
    }


    /**
     * Gets the PIDF values for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The PIDF values for the motor.
     */

    public double[] getPIDFArrayByID(int deviceID) {
        return new double[] {getSparkBaseByID(deviceID).getPIDController().getP(), getSparkBaseByID(deviceID).getPIDController().getI(), getSparkBaseByID(deviceID).getPIDController().getD(), getSparkBaseByID(deviceID).getPIDController().getFF()};
    }

    /**
     * Sets the Izone for the motors in the group.
     * 
     * @param Izone The Izone to set for the motors.
     */

    public void setIzone(int Izone) {
        deviceIDToSparkBase.values().forEach(sparkBase -> sparkBase.getPIDController().setIZone(Izone));
    }

    /**
     * Sets the Izone for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @param Izone The Izone to set for the motor.
     */

    public void setIzoneByID(int deviceID, int Izone) {
        getSparkBaseByID(deviceID).getPIDController().setIZone(Izone);
    }


    /**
     * Gets the Izone for the motors in the group.
     * 
     * @return The Izone for the motors in the group.
     * @throws IllegalStateException        If no CANSparkBase objects are found.
     * @throws UnsupportedOperationException If the value of one or more specific CANSparkBase objects was changed.
     */

    public int getIzone() {
        Iterator<CANSparkBase> iterator = deviceIDToSparkBase.values().iterator();
        if (!iterator.hasNext()) {
            throw new IllegalStateException("No CANSparkBase objects found.");
        }

        int firstIzone = iterator.next().getPIDController().getIZone();
        while (iterator.hasNext()) {
            if (iterator.next().getPIDController().getIZone() != firstIzone) {
                throw new UnsupportedOperationException(
                        "The value of one or more specific CANSparkBase objects was changed, therefore you must use the .getIzone method of this class.");
            }
        }

        return firstIzone;
    }

    /**
     * Gets the average Izone for the motors in the group.
     * 
     * @return The average Izone for the motors in the group.
     */

    public int getAverageIzone() {
        try {
            this.getIzone();
        } catch (UnsupportedOperationException e) {
            int sumIzone = 0;
            for (CANSparkBase sparkBase : deviceIDToSparkBase.values()) {
                sumIzone += sparkBase.getPIDController().getIZone();
            }
            return sumIzone / deviceIDToSparkBase.values().size();
        }
    }

    /**
     * Gets the Izone for the motor in the group based on the device ID.
     * 
     * @param deviceID The device ID of the motor.
     * @return The Izone for the motor.
     */

    public int getIzoneByID(int deviceID) {
        return getSparkBaseByID(deviceID).getPIDController().getIZone();
    }

    
     






}