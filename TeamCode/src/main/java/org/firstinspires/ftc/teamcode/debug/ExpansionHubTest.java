package org.firstinspires.ftc.teamcode.debug;

import com.disnodeteam.dogecv.math.Line;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utilities.hardware.ExpansionHub;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by LeviG on 2/17/2019.
 */
@TeleOp(name = "Expansion Hub Test")
@Disabled
public class ExpansionHubTest extends LinearOpMode {

    AnalogInput input;
    ExpansionHub hub;

    @Override
    public void runOpMode() throws InterruptedException{
        input = hardwareMap.get(AnalogInput.class, "left");
        hotswapHardwareMap();
        getOpModeManager().registerListener(new OpModeNotifications());
        hub = hardwareMap.get(ExpansionHub.class, "Left");
        waitForStart();
        try {
            Field controllerField = AnalogInput.class.getDeclaredField("controller");
            controllerField.setAccessible(true);
            AnalogInputController controller = (AnalogInputController) controllerField.get(input);

        } catch (Exception e) {

        }
        hub.setLedColor(255, 0, 255);
        Thread.sleep(200000);
    }

    static void hotswapHardwareMap()
    {
        HardwareMap map = getHardwareMap();

        addExpansionHubExForEachLynxModule(map);
    }

    static OpModeManagerImpl getOpModeManager()
    {
        return OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity());
    }

    private static void addExpansionHubExForEachLynxModule(HardwareMap map)
    {
        //-----------------------------------------------------------------------------------
        // LynxModules --+> ExpansionHubEx
        //-----------------------------------------------------------------------------------

        HashMap<String, ExpansionHub> enhancedLynxModulesToInject = new HashMap<>();

        for (LynxModule module : map.getAll(LynxModule.class))
        {
            if (!hwMapContainsEnhancedModule(module))
            {
                enhancedLynxModulesToInject.put(getHwMapName(module), new org.firstinspires.ftc.teamcode.utilities.hardware.ExpansionHub(module));
            }
        }

        for (Map.Entry<String, ExpansionHub> entry : enhancedLynxModulesToInject.entrySet())
        {
            map.put(entry.getKey(), entry.getValue());
        }
    }

    static HardwareMap getHardwareMap()
    {
        return getOpModeManager().getHardwareMap();
    }

    static String getHwMapName(HardwareDevice device)
    {
        return getHardwareMap().getNamesOf(device).iterator().next();
    }

    private static boolean hwMapContainsEnhancedModule(LynxModule module)
    {
        for(ExpansionHub enhancedModule : getHardwareMap().getAll(ExpansionHub.class))
        {
            //TODO: can we just replace with an '=='?
            if(module.getModuleAddress() == enhancedModule.getStandardModule().getModuleAddress())
            {
                return true;
            }
        }

        return false;
    }

    private static void removeExpansionHubExForEachLynxModule(HardwareMap map)
    {
        //-----------------------------------------------------------------------------------
        // Remove ExpHbExs
        //-----------------------------------------------------------------------------------

        HashMap<String, ExpansionHub> enhancedLynxModulesToRemove = new HashMap<>();

        for (ExpansionHub module : map.getAll(ExpansionHub.class))
        {
            enhancedLynxModulesToRemove.put(getHwMapName(module), module);
        }

        if (!enhancedLynxModulesToRemove.isEmpty())
        {
            for (Map.Entry<String, ExpansionHub> entry : enhancedLynxModulesToRemove.entrySet())
            {
                map.remove(entry.getKey(), entry.getValue());
            }
        }
    }

    static void deswapHardwareMap()
    {
        HardwareMap map = getHardwareMap();

        removeExpansionHubExForEachLynxModule(map);
    }

    private static class OpModeNotifications implements OpModeManagerNotifier.Notifications
    {
        @Override
        public void onOpModePreInit(OpMode opMode)
        {
            /*
             * I'd LOVE to find a way to register this lister automatically so that we
             * can hotswap the hardware map here instead of making the user call init(),
             * but so far I've been unable to find a way to do that.
             */

            //Utils.hotswapHardwareMap();
        }

        @Override
        public void onOpModePreStart(OpMode opMode)
        {

        }

        @Override
        public void onOpModePostStop(OpMode opMode)
        {
            deswapHardwareMap();
            getOpModeManager().unregisterListener(this);
        }
    }

}
