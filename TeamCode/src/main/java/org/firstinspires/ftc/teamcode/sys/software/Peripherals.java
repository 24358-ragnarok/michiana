package org.firstinspires.ftc.teamcode.sys.software;

import android.graphics.Color;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.teamcode.config.Settings;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Peripherals {
    public final SoundPlayer sfx = SoundPlayer.getInstance();
    private final LynxModule ctrl;
    private final LynxModule exp;
    private final HardwareMap hardwareMap;


    public Peripherals(HardwareMap hardwareMap) {
        this.ctrl = PhotonCore.CONTROL_HUB;
        this.exp = PhotonCore.EXPANSION_HUB;
        this.hardwareMap = hardwareMap;

        setupHubs();
    }

    private void setupHubs() {
        ctrl.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        exp.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();
        LynxModule.blinkerPolicy = new BlinkyBlinky();
        setHubColors(PresetColor.RAINBOW);
    }

    public void playSound(File f) {
        if (Settings.Flags.SFX && Settings.Flags.DEBUG) {
            sfx.play(hardwareMap.appContext, f, 1.0f, 0, 1.0f);
        }
    }

    public void setHubColors(PresetColor c) {
        ArrayList<Blinker.Step> p = new ArrayList<>();
        int maxPatternLength = ctrl.getBlinkerPatternMaxLength();
        switch (c) {
            case RED:
                ctrl.setConstant(Color.RED);
                exp.setConstant(Color.RED);
                break;
            case PURPLE:
                ctrl.setConstant(Color.MAGENTA);
                exp.setConstant(Color.MAGENTA);
                break;
            case BLUE:
                ctrl.setConstant(Color.BLUE);
                exp.setConstant(Color.BLUE);
                break;
            case DEAD:
                ctrl.setConstant(Color.BLACK);
                exp.setConstant(Color.BLACK);
                break;
            case RAINBOW:
            default:
                int targetLength = Math.max(1, maxPatternLength);

                int totalCycleTimeMs = 500;
                int stepDuration = totalCycleTimeMs / targetLength;

                for (int i = 0; i < targetLength; i++) {
                    // Map the current step to a degree on the 360 degree color wheel
                    float hue = (i * 360f) / targetLength;

                    // Convert HSV to Android Color (Full Saturation and Value)
                    int color = Color.HSVToColor(new float[]{hue, 1f, 1f});

                    p.add(new Blinker.Step(color, stepDuration, TimeUnit.MILLISECONDS));
                }
                ctrl.setPattern(p);
                exp.setPattern(p);
                break;
            case MORSE:

                String morseString = new String(
                        android.util.Base64.decode("Li0gLi4uIC4tLS4gLS4tLSAtLg==", android.util.Base64.DEFAULT));

                // Timing (ms)
                int dot = 150;
                int dash = dot * 3;
                int gap = dot;
                int letterGap = dot * 3;

                int red = Color.RED;

                p.clear();
                for (String letter : morseString.split(" ")) {
                    for (int i = 0; i < letter.length(); i++) {
                        int dur = (letter.charAt(i) == '.') ? dot : dash;
                        p.add(new Blinker.Step(red, dur, TimeUnit.MILLISECONDS));
                        p.add(new Blinker.Step(Color.BLACK, gap, TimeUnit.MILLISECONDS));
                    }
                    // replace last gap with longer letter gap
                    p.set(p.size() - 1, new Blinker.Step(Color.BLACK, letterGap, TimeUnit.MILLISECONDS));
                }
                ctrl.setPattern(p);
                exp.setPattern(p);
                break;
        }
    }

    public enum PresetColor {
        RAINBOW,
        PURPLE,
        RED,
        BLUE,
        DEAD,
        MORSE
    }


    private static class BlinkyBlinky implements LynxModule.BlinkerPolicy {
        private static final int DEFAULT_CYCLE_MS = 2500;

        @Override
        public List<Blinker.Step> getIdlePattern(LynxModule lynxModule) {
            int maxSteps = Math.max(4, lynxModule.getBlinkerPatternMaxLength());
            int stepDuration = DEFAULT_CYCLE_MS / maxSteps;
            List<Blinker.Step> steps = new ArrayList<>();

            for (int i = 0; i < maxSteps; i++) {
                float hue = (i * 360f) / maxSteps;
                int color = Color.HSVToColor(new float[]{hue, 1f, 1f});
                steps.add(new Blinker.Step(color, stepDuration, TimeUnit.MILLISECONDS));
            }
            return steps;
        }

        @Override
        public List<Blinker.Step> getVisuallyIdentifyPattern(LynxModule lynxModule) {
            List<Blinker.Step> steps = new ArrayList<>();
            int stepDuration = 150;
            steps.add(new Blinker.Step(Color.WHITE, stepDuration, TimeUnit.MILLISECONDS));
            steps.add(new Blinker.Step(Color.BLACK, stepDuration, TimeUnit.MILLISECONDS));
            return steps;
        }
    }
}
