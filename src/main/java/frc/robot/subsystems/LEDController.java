package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {

    private PWM blinkin;
    private Colours setColour;
    

    public LEDController(){
        blinkin = new PWM(0);
        blinkin.setPosition(0);
     }

     public Command HoldSetColour(){
        return this.runOnce(() -> blinkin.setPosition(TranslateColour(setColour)));
     }

     public void periodic(){
         HoldSetColour();
     }

     private int TranslateColour(Colours colour){
         if(colour == Colours.RED) return red;
         if(colour == Colours.BLUE) return blue;
         if(colour == Colours.STARTUPPATTERN) return StartUpPatern;
         if(colour == Colours.YELLOW) return yellow;
         if(colour == Colours.PURPLE) return purple;
         if(colour == Colours.GREEN) return green;
         if(colour == Colours.PURPLEFLASHING) return FlashingPrple;
         if(colour == Colours.BlueFlashing) return FlashingBlue;
         if(colour == Colours.ERROR_YELLOw) return ErrorYellow;
         if(colour == Colours.GREEN_WHITE_PATTERN) return greenAndWhitePattern;
         if(colour == Colours.ERROR_RED) return ErroRed;
         if(colour == Colours.COOL_PATTERN) return CoolPatern;                      // rainbow
         else return 0;   
     }

   private final int red = 1075;
   private final int blue = 1085;
   private final int StartUpPatern = 1305;
   private final int yellow = 1835;
   private final int purple = 1955;
   private final int green  = 1855;
   private final int FlashingPrple = 1975;
   private final int FlashingBlue = 1295;
   private final int ErrorYellow = 1465;
   private final int greenAndWhitePattern = 1205;
   private final int ErroRed = 1185;
   private final int CoolPatern = 1115;


   public enum Colours{
      RED,
      BLUE,
      STARTUPPATTERN,
      YELLOW,
      PURPLE,
      GREEN,
      PURPLEFLASHING,
      BlueFlashing,
      ERROR_YELLOw,
      ERROR_RED,
      GREEN_WHITE_PATTERN,
      COOL_PATTERN
   }

   public Command setColour(Colours c){
        SmartDashboard.putNumber("color", TranslateColour(c));
        return this.runOnce(() -> setColour = c);
   }

}
