package frc.robot;

import org.team217.*;

/**
 * Checks the state of various controls to determine if the bot should run presets.
 * 
 * @author ThunderChickens 217
 */
public class PresetState {
    /** Variable that contains information on the current arm preset state. */
    public static enum Preset {
        Manual,
        Low,
        Mid,
        High,
        HighBall,
        Ball,
        Climb
    }

    private static Preset presetState = Preset.Manual;
    static Preset lastPreset = Preset.Manual;

    /** Returns {@code true} if all controls, excluding `oper.getPOV()` and `getWristStatus()`, permit presets. */
    public static boolean getStatus() {
        return getArmStatus() && getElevStatus() && getTelescopeStatus();
    }

    /** Returns {@code true} if the arm controls permit presets. */
    public static boolean getArmStatus() {
        double armSpeed = Num.deadband(Robot.m_oi.oper.getRawAxis(5), 0.1);
        return armSpeed == 0;
    }

    /** Returns {@code true} if the wrist controls permit presets. */
    public static boolean getWristStatus() {
        return !(Robot.m_oi.rightTriggerOper.get() || Robot.m_oi.leftTriggerOper.get());
    }

    /** Returns {@code true} if the elevator controls permit presets. */
    public static boolean getElevStatus() {
        double leftAnalog = Num.deadband(Robot.m_oi.oper.getY(), 0.1);
        return leftAnalog == 0;
    }

    /** Returns {@code true} if the telescope controls permit presets. */
    public static boolean getTelescopeStatus() {
        return !(Robot.m_oi.squareOper.get() || Robot.m_oi.xOper.get());
    }

    /** Returns {@code true} if `oper.getPOV()` permits presets. */
    public static boolean getPOVStatus() {
        return (Robot.m_oi.oper.getPOV() != -1 && Robot.m_oi.oper.getButtonCount() != 0) || Robot.m_oi.triangleDriver.get() || Robot.m_oi.psButtonOper.get();
    }

    public static Preset getPreset() {
        if (Robot.m_oi.oper.getPOV() == 180) {
            presetState = Preset.Low;
        }
        else if (Robot.m_oi.oper.getPOV() == 270) {
            presetState = Preset.Mid;
        }
        else if (Robot.m_oi.oper.getPOV() == 0) {
            presetState = Preset.High;
        }
        else if (Robot.m_oi.oper.getPOV() == 90) {
            presetState = Preset.Ball;
        }
        else if (Robot.m_oi.psButtonOper.get()) {
            presetState = Preset.HighBall;
        }
        else if (Robot.m_oi.triangleDriver.get()) {
            presetState = Preset.Climb;
        }
        else if (!getStatus()) {
            presetState = Preset.Manual;
        }
        
        return presetState;
    }

    public static Preset getPresetState() {
        getPreset();

        if (presetState.equals(Preset.Manual)) {
            if (RobotMap.rightArm.getPosition() > 45 && RobotMap.telescope.getEncoder() >= 7000) {
                lastPreset = Preset.High;
            }
            else {
                lastPreset = Preset.Low;
            }
            return presetState;
        }

        switch(presetState) {
        case High:
        case HighBall:
            if (RobotMap.telescope.getEncoder() >= 7000 && !lastPreset.equals(Preset.High) && !lastPreset.equals(Preset.HighBall)) {
                lastPreset = Preset.Mid;
                return Preset.Mid;
            }

            if (!lastPreset.equals(Preset.High) && !lastPreset.equals(Preset.HighBall) && (!lastPreset.equals(Preset.Mid) || !Robot.kArmSubsystem.atPreset)) {
                lastPreset = Preset.Mid;
                return Preset.Mid;
            }
            break;
        case Climb:
            break;
        case Low:
            if (lastPreset.equals(Preset.Ball) && RobotMap.leftElevator.getEncoder() < 3000) {
                return Preset.Mid;
            }
        default:
            if (RobotMap.telescope.getEncoder() >= 7000 && !lastPreset.equals(Preset.Low)) {
                lastPreset = Preset.Mid;
                return Preset.Mid;
            }

            if (lastPreset.equals(Preset.High) || lastPreset.equals(Preset.HighBall) || (lastPreset.equals(Preset.Mid) && !Robot.kTelescopeSubsystem.atPreset)) {
                lastPreset = Preset.Mid;
                return Preset.Mid;
            }
            break;
        }

        lastPreset = presetState;
        return presetState;
    }
}