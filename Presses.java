package org.firstinspires.ftc.teamcode;

public class Presses{ 
    private boolean toggledVariable=false;
    private boolean wasPressedVariable=false;
    private boolean wasPressedForToggledVariable=false;

    public boolean released(boolean inputBoolean){
        if (wasPressedVariable!=inputBoolean && wasPressedVariable==true){
            wasPressedVariable=inputBoolean;
            return true;
        } else {
            wasPressedVariable=inputBoolean;
            return false;
        }
    }

    public boolean change(boolean inputBoolean){
        if (wasPressedVariable!=inputBoolean){
            wasPressedVariable=inputBoolean;
            return true;
        } else {
            wasPressedVariable=inputBoolean;
            return false;
        }
    }

    public boolean pressed(boolean inputBoolean){
        
        if (wasPressedVariable!=inputBoolean && wasPressedVariable==false){
            wasPressedVariable=inputBoolean;
            return true;
        
        } else {
            wasPressedVariable=inputBoolean;
            return false;
        }
        
    }

    public void setToggleFalse(){
        toggledVariable=false;
    }

    public void setToggleTrue(){
        toggledVariable=true;
    }

    public boolean toggle (boolean inputBoolean){
        if (pressed(inputBoolean) && toggledVariable == false){
            toggledVariable=true;
        } 
        if (pressed(inputBoolean) && toggledVariable == true){
            toggledVariable=false;
        }
        return toggledVariable;
    }
}