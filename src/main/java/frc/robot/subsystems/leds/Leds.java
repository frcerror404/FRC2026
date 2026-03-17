package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

  private final CANdle candle;

  // Skip the 8 onboard LEDs
  private static final int STRIP_START = 8;

  // Change this to match your strip length
  private static final int STRIP_LENGTH = 200;

  private final Timer timer = new Timer();

  private double duration = 0;
  private boolean running = false;

  private int r = 0;
  private int g = 0;
  private int b = 0;

  public Leds(int canID) {
    candle = new CANdle(canID, "2026 Swerve");
  }

  // Starts a countdown progress bar.

  public void startCountdown(double seconds, int r, int g, int b) {
    this.duration = seconds;
    this.r = r;
    this.g = g;
    this.b = b;

    timer.reset();
    timer.start();
    running = true;
  }

  // Turns the LED strip off.

  public void turnOff() {
    candle.setControl(new SolidColor(STRIP_START, STRIP_LENGTH).withColor(new RGBWColor(0, 0, 0)));
    running = false;
  }

  @Override
  public void periodic() {

    if (!running) return;

    double elapsed = timer.get();
    double remaining = Math.max(0, duration - elapsed);

    if (remaining <= 0) {
      turnOff();
      return;
    }

    double percentRemaining = remaining / duration;

    int ledsToShow = (int) (STRIP_LENGTH * percentRemaining);

    // Turn all LEDs off first
    candle.setControl(new SolidColor(STRIP_START, STRIP_LENGTH).withColor(new RGBWColor(0, 0, 0)));

    // Turn on the progress LEDs
    candle.setControl(new SolidColor(STRIP_START, ledsToShow).withColor(new RGBWColor(r, g, b)));
  }
}
