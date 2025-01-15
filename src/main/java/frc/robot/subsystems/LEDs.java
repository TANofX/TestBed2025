/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The CANdle MultiAnimation example demonstrates using multiple animations with CANdle.
 * This example has the robot using a Command Based template to control the CANdle.
 * 
 * This example uses:
 * - A CANdle wired on the CAN Bus, with a 5m led strip attached for the extra animatinos.
 * 
 * Controls (with Xbox controller):
 * Right Bumper: Increment animation
 * Left Bumper: Decrement animation
 * Start Button: Switch to setting the first 8 LEDs a unique combination of colors
 * POV Right: Configure maximum brightness for the CANdle
 * POV Down: Configure medium brightness for the CANdle
 * POV Left: Configure brightness to 0 for the CANdle
 * POV Up: Change the direction of Rainbow and Fire, must re-select the animation to take affect
 * A: Print the VBat voltage in Volts
 * B: Print the 5V voltage in Volts
 * X: Print the current in amps
 * Y: Print the temperature in degrees C
 * 
 * Supported Version:
 * 	- CANdle: 22.1.1.0
 */

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class LEDs extends AdvancedSubsystem {
    private final int LEDS_PER_ANIMATION = 70;
    public final CANdle m_candle = new CANdle(Constants.LEDs.CANdleID, "rio");
    private int m_candleChannel = 0;
    private boolean m_clearAllAnims = false;
    private boolean m_last5V = true;
    private boolean m_animDirection = false;
    private boolean m_setAnim = false;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        OneColorRed,
        OneColorBlue,
        OneColorGreen,
        Empty, OneColorOrange, OneColorYellow
    }

    private AnimationTypes m_currentAnimation;

    public LEDs() { // void?
        // this.joystick = joy;
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.v5Enabled = true;
        configAll.vBatOutputMode = VBatOutputMode.Off;
        configAll.enableOptimizations = true;
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
        changeAnimation(AnimationTypes.Empty);
    }

    public void toggle5VOverride() {
        System.out.println("State is: " + m_last5V);
        m_candle.configV5Enabled(m_last5V);
        m_last5V = !m_last5V;
    }

    public void toggleAnimDirection() {
        m_animDirection = !m_animDirection;
    }

    public int getMaximumAnimationCount() {
        return m_candle.getMaxSimultaneousAnimationCount();
    }

    public void incrementAnimation() {
        switch (m_currentAnimation) {
            case ColorFlow:
                changeAnimation(AnimationTypes.Fire);
                break;
            case Fire:
                changeAnimation(AnimationTypes.Larson);
                break;
            case Larson:
                changeAnimation(AnimationTypes.Rainbow);
                break;
            case Rainbow:
                changeAnimation(AnimationTypes.RgbFade);
                break;
            case RgbFade:
                changeAnimation(AnimationTypes.SingleFade);
                break;
            case SingleFade:
                changeAnimation(AnimationTypes.Strobe);
                break;
            case Strobe:
                changeAnimation(AnimationTypes.Twinkle);
                break;
            case Twinkle:
                changeAnimation(AnimationTypes.TwinkleOff);
                break;
            case TwinkleOff:
                changeAnimation(AnimationTypes.Empty);
                break;
            case Empty:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
            case SetAll:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
            default:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
        }
    }

    public void decrementAnimation() {
        switch (m_currentAnimation) {
            case ColorFlow:
                changeAnimation(AnimationTypes.Empty);
                break;
            case Fire:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
            case Larson:
                changeAnimation(AnimationTypes.Fire);
                break;
            case Rainbow:
                changeAnimation(AnimationTypes.Larson);
                break;
            case RgbFade:
                changeAnimation(AnimationTypes.Rainbow);
                break;
            case SingleFade:
                changeAnimation(AnimationTypes.RgbFade);
                break;
            case Strobe:
                changeAnimation(AnimationTypes.SingleFade);
                break;
            case Twinkle:
                changeAnimation(AnimationTypes.Strobe);
                break;
            case TwinkleOff:
                changeAnimation(AnimationTypes.Twinkle);
                break;
            case Empty:
                changeAnimation(AnimationTypes.TwinkleOff);
                break;
            case SetAll:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
            case OneColorRed:
                changeAnimation(AnimationTypes.SetAll);
                break;
            case OneColorBlue:
                changeAnimation(AnimationTypes.SetAll);
                break;
            case OneColorGreen:
                changeAnimation(AnimationTypes.SetAll);
                break;
            default:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
        }
    }

    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() {
        return m_candle.getBusVoltage();
    }

    public double get5V() {
        return m_candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return m_candle.getCurrent();
    }

    public double getTemperature() {
        return m_candle.getTemperature();
    }

    public void configBrightness(double percent) {
        m_candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        m_candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        m_candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        m_candle.configStatusLedState(offWhenActive, 0);
    }

    public void changeAnimation(AnimationTypes toChange) {
        // Temporary disable CANdle animation
        // Ignore the complaint
        if(true) return;
        m_candle.clearAnimation(m_candleChannel);
        m_currentAnimation = toChange;

        switch (toChange) {
            case Empty:
                m_candleChannel = 0;
                m_toAnimate = new RainbowAnimation(0.75, 0.7, LEDS_PER_ANIMATION, m_animDirection, 8);
                break;

            default:
            case SetAll:
                m_toAnimate = null;
                break;

            case OneColorRed:
                m_candleChannel = 1;
                m_toAnimate = new TwinkleAnimation(130, 0, 0, 0, 0.0, LEDS_PER_ANIMATION, TwinklePercent.Percent76, 8);
                break;
            case OneColorOrange:
                m_candleChannel = 2;
                m_toAnimate = new TwinkleAnimation(237, 130, 24, 0, 0.75, LEDS_PER_ANIMATION, TwinklePercent.Percent76, 8);
                break;

            case OneColorBlue:
                m_candleChannel = 3;
                m_toAnimate = new TwinkleAnimation(0, 0, 130, 0, 0.0, LEDS_PER_ANIMATION,  TwinklePercent.Percent76, 8);
                break;

            case OneColorGreen:
                m_candleChannel = 4;
                m_toAnimate = new TwinkleAnimation(0, 130, 0, 0, 0.0, LEDS_PER_ANIMATION, TwinklePercent.Percent76, 8);
                break;

            case OneColorYellow:
                m_candleChannel = 5;
                m_toAnimate = new TwinkleAnimation(200, 200, 0, 0, 0.75, LEDS_PER_ANIMATION,  TwinklePercent.Percent76, 8);
                break;

        }
 //       System.out.println("Changed to " + m_currentAnimation.toString());
    }

    public void clearAllAnims() {
        m_clearAllAnims = true;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_toAnimate == null) {
            if (!m_setAnim) {
                // set LED strip
                m_candle.setLEDs(255, 0, 128, 128, 8, LEDS_PER_ANIMATION);
                /* Only setLEDs once, because every set will transmit a frame */
                m_candle.setLEDs(255, 255, 255, 0, 0, 1);
                m_candle.setLEDs(255, 255, 0, 0, 1, 1);
                m_candle.setLEDs(255, 0, 255, 0, 2, 1);
                m_candle.setLEDs(255, 0, 0, 0, 3, 1);
                m_candle.setLEDs(0, 255, 255, 0, 4, 1);
                m_candle.setLEDs(0, 255, 0, 0, 5, 1);
                m_candle.setLEDs(0, 0, 0, 0, 6, 1);
                m_candle.setLEDs(0, 0, 255, 0, 7, 1);
                m_setAnim = true;
            }
         } else {
        //     m_toAnimate.setSpeed((joystick.getRightY() + 1.0) / 2.0);
            m_candle.animate(m_toAnimate, m_candleChannel);
            m_setAnim = false;
        }
        // m_candle.modulateVBatOutput(joystick.getRightY());

        if (m_clearAllAnims) {
            m_clearAllAnims = false;
            for (int i = 0; i < 10; ++i) {
                m_candle.clearAnimation(i);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.waitSeconds(0.1);
    }
}
