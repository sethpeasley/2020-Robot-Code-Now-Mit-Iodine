/*
 * this class is for Controlling the ControlPanel
 * @author Nicholas Blackburn ,Luke Crum
 */
package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanel extends SubsystemBase
{

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);

    private final Victor m_CpMotor = new Victor(Constants.k_CPMotorPort);

    private int targetCounter, colorCounter;

    private final ColorMatch m_colorMatcher = new ColorMatch();
    private ColorState targetColor, currentColor, observedColor, lastColor;

    public boolean DisplayColor = false;

    // This Enums are used for the Control system
    private enum ControlPanelState
    {
        INIT,
        HOLD,
        INIT_ROTATION_CONTROL,
        ROTATION_CONTROL,
        SEES_TARGET_COLOR_ROTATION,
        TARGET_COLOR_LEFT_VIEW,
        INIT_POSITION_CONTROL,
        POSITION_CONTROL,
        SEES_TARGET_COLOR_POSITION,
        ERROR
    }

    // Color enums for Color Sensor
    private enum ColorState
    {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        NONE
    }

    private ControlPanelState currentState;

    //  Functions Runs Once every Command run
    public ControlPanel()
    {
        m_colorMatcher.addColorMatch(Constants.k_BlueTarget);
        m_colorMatcher.addColorMatch(Constants.k_GreenTarget);
        m_colorMatcher.addColorMatch(Constants.k_RedTarget);
        m_colorMatcher.addColorMatch(Constants.k_YellowTarget);
        currentState = ControlPanelState.INIT;
        setControlPanelState(ControlPanelState.INIT);
    }

    /* Get the current color from the color sensor */
    public Color getCurrentColor()
    {
        return m_colorSensor.getColor();
    }

    /* get the color assigned to our alliance by the field management system */
    public char getFMSColor()
    {
        return DriverStation.getInstance().getGameSpecificMessage().charAt(0);
    }

    /* sets Control Panel motor speed for controller */
    public void runControlPanel(double speed)
    {
        m_CpMotor.setSpeed(speed);
    }

    /* Stops ControlPanel motor */
    public void stopControlPanel()
    {
        m_CpMotor.set(0);
    }

    /*Returns Current Color  for use in DashBoard*/
    public boolean isCurrentColor()
    {
        return DisplayColor;
    }

    /**Returns Current Color Seen By Field Sensor   */
    public ColorState getCurrentCPColor()
    {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        // Checks if The FMS Color = blue on te Robot Color sensor
        if (match.color == Constants.k_BlueTarget)
        {
            return ColorState.BLUE;
        }

        // Checks if The FMS Color = RED on te Robot Color sensor
        else if (match.color == Constants.k_RedTarget)
        {
            return ColorState.RED;
        }

        // Checks if The FMS Color = Green on te Robot Color sensor
        else if (match.color == Constants.k_GreenTarget)
        {
            return ColorState.GREEN;

        }

        // Checks if The FMS Color = Yellow on te Robot Color sensor
        else if (match.color == Constants.k_YellowTarget)
        {
            return ColorState.YELLOW;
        }

        else
        {
            //System.out.println("NONE");
            return ColorState.NONE;
        }
    }

    private ColorState getPositionTargetColor()
    {
        char fmscolor = getFMSColor();

        if(fmscolor == 'R')
        {
            return ColorState.BLUE;
        }
        else if(fmscolor == 'B')
        {
            return ColorState.RED;
        }
        else if(fmscolor == 'G')
        {
            return ColorState.YELLOW;
        }
        else if(fmscolor == 'Y')
        {
            return ColorState.GREEN;
        }
        else
        {
            return ColorState.NONE;
        }
    }

    public void setControlPanelState(ControlPanelState state)
    {
        switch(state)
        {
            case INIT:
                stopControlPanel();
                targetCounter = 0;
                lastColor = ColorState.NONE;
                colorCounter = 0;
                currentColor = ColorState.NONE;
                currentState = ControlPanelState.INIT;
            break;
            case HOLD:
                stopControlPanel();
                currentState = ControlPanelState.HOLD;
            break;
            case INIT_ROTATION_CONTROL:
                if(currentColor != ColorState.NONE)
                {
                    targetColor = getCurrentCPColor();
                    runControlPanel(Constants.k_ControlPanelSpeed);
                    setControlPanelState(ControlPanelState.ROTATION_CONTROL);
                }
                else
                {
                    //DriverStation.reportWarning("No color found, cancelling rotation control",true); //TODO: Dashboard message
                    setControlPanelState(ControlPanelState.HOLD);
                }
            break;
            case ROTATION_CONTROL:
                runControlPanel(Constants.k_ControlPanelSpeed);
                currentState = ControlPanelState.ROTATION_CONTROL;
            break;
            case SEES_TARGET_COLOR_ROTATION:
                currentState = ControlPanelState.SEES_TARGET_COLOR_ROTATION;
            break;
            case TARGET_COLOR_LEFT_VIEW:
                targetCounter++;
                setControlPanelState(ControlPanelState.ROTATION_CONTROL);
            break;
            case INIT_POSITION_CONTROL:
                if(currentColor != ColorState.NONE)
                {
                    targetColor = getPositionTargetColor();
                    setControlPanelState(ControlPanelState.POSITION_CONTROL);
                }
                else
                {
                    //DriverStation.reportWarning("No color found, cancelling position control.",true); //TODO: Dashboard Message
                    setControlPanelState(ControlPanelState.HOLD);
                }
            break;
            case POSITION_CONTROL:
                runControlPanel(Constants.k_ControlPanelSpeed);
                currentState = ControlPanelState.POSITION_CONTROL;
            break;
            case SEES_TARGET_COLOR_POSITION:
                stopControlPanel();
                setControlPanelState(ControlPanelState.HOLD);
            break;
            case ERROR:
            default:
                // DriverStation.reportError("Error in ControlPanel, you shouldn't see this. Cringe.",true);
                currentState = ControlPanelState.ERROR;
            break;
        }
    }

    public void CPControl()
    {
        observedColor = getCurrentCPColor();
        if(observedColor == lastColor)
        {
            if(colorCounter < 5)
            {
                colorCounter++;
                lastColor = observedColor;
            }
            else if(colorCounter >= 5)
            {
                currentColor = observedColor;
                lastColor = observedColor;
            }
        }
        else if(observedColor != lastColor)
        {
            colorCounter = 0;
            lastColor = observedColor;
        }
        if(currentColor == targetColor && currentState == ControlPanelState.ROTATION_CONTROL && targetCounter < Constants.k_MaxCPTicks)
        {
            setControlPanelState(ControlPanelState.SEES_TARGET_COLOR_ROTATION);
        }
        if(currentColor != targetColor && currentState == ControlPanelState.SEES_TARGET_COLOR_ROTATION)
        {
            setControlPanelState(ControlPanelState.TARGET_COLOR_LEFT_VIEW);
        }
        if(currentColor == targetColor && currentState == ControlPanelState.ROTATION_CONTROL && targetCounter >= Constants.k_MaxCPTicks)
        {
            setControlPanelState(ControlPanelState.HOLD);
        }
        if(currentState == ControlPanelState.POSITION_CONTROL && getCurrentCPColor() == targetColor)
        {
            setControlPanelState(ControlPanelState.SEES_TARGET_COLOR_POSITION);
        }
    }

    @Override
    public void periodic()
    {
    }
}