package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

// import static edu.wpi.first.units.Units.Percent;
// import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

public class LEDS extends SubsystemBase{
    private static LEDS led = null;

    //Objects
    private AddressableLED strip;
    private AddressableLEDBuffer Buffer;
    // private AddressableLEDBufferView leftStrip;
    // private AddressableLEDBufferView rightStrip;
    private LEDPattern base;
    private LEDPattern pattern;
    // private TimeUnit Seconds;

// colours: 
// yellow - intake 
// blue - teleop/drive 
// orange - climb
// violet - auton
// green - charging for shooter
// red - shooter is ready/shooting

    //constructor
    private LEDS(){
        //Object creations
        strip = new AddressableLED(0); // set proper pwm location
        Buffer = new AddressableLEDBuffer(100); //set proper amount of leds

        // leftStrip = Buffer.createView(0,49);
        // rightStrip = Buffer.createView(50,99).reversed();

        //LED initializations
        strip.setLength(Buffer.getLength());
        strip.setData(Buffer);  
        strip.start();
    }

    //Blink effect during teleop
    public void teleopBlink(){
        //Initialization for this pattern
        // change pattern
        base = LEDPattern.solid(Color.kMediumBlue);
        pattern = base.blink(Seconds.of(1));
        //Apply effect
        pattern.applyTo(Buffer);
        strip.setData(Buffer);
    }
    //Solid colour during intake
    public void intakeSolid() {
        // Initialization for this pattern
        base = LEDPattern.solid(Color.kYellow);
        //Apply effect
        base.applyTo(Buffer);
        strip.setData(Buffer);
    }
    //Wave effect during climb
    public void climbWave() {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkOrange, Color.kBisque);
        pattern = base.breathe(Seconds.of(2)); //.progressMaskLayer(() -> getheightofclimb() / getmaxheightofclimb());
        //Apply effect
        pattern.applyTo(Buffer);
        strip.setData(Buffer);
    }

    public void shooterChargingWave() {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kForestGreen, Color.kLightSeaGreen);
        pattern = base.breathe(Seconds.of(2));
        //Apply effect
        pattern.applyTo(Buffer);
        strip.setData(Buffer);
    }

    public void shooterReadyBlink() {
        base = LEDPattern.solid(Color.kRed);
        pattern = base.blink(Seconds.of(1));
        //Apply effect
        pattern.applyTo(Buffer);
        strip.setData(Buffer);
    }

    //Wave effect during auton
    public void autonWave(){
        //Initialization for this pattern
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kViolet, Color.kWhiteSmoke);
        pattern = base.breathe(Seconds.of(2));
        //Apply effect
        pattern.applyTo(Buffer);
        strip.setData(Buffer);
    }

    public static LEDS getInstance(){
        if (led == null){
            led = new LEDS();
        }
        return led;
    }
}