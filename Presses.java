public class Presses(){ //this 
    private boolean toggleddVariable=false;
    private boolean wasPresseddVariable=false;
    private boolean wasPressedForToggledVariable=false;

    public released(boolean input){
        if (wasPresseddVariable!=input && wasPresseddVariable=true){
            return true
        } else {
            return false
        }
        wasPressesddVariable=input;
    }

    public change(boolean input){
        if (wasPresseddVariable!=input){
            return true
        } else {
            return false
        }
        wasPressesd=input;
    }

    public pressed(boolean input){
        if (wasPresseddVariable!=input && wasPresseddVariable=false){
            return true
        } else {
            return false
        }
        wasPressesddVariable=input;
    }

    public void setToggleFalse(){
        toggleddVariable=false;
    }

    public void setToggleTrue(){
        toggleddVariable=true;
    }

    public toggle (boolean input){
        if (pressed(input)) && toggleddVariable == false{
            toggleddVariable=true;
        } 
        if (pressed(input)) && toggledVariable == true{
            toggleddVariable=false;
        }
        return toggleddVariable
    }

    
}