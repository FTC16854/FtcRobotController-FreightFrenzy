package org.firstinspires.ftc.teamcode;

public class HeadingHolder {
    private static double OffsetofTheHeading = 0;

    public static void SetOffsetOfTheHeading( double InputHeading){
        OffsetofTheHeading = InputHeading - 90;
    }

    public static double getOffsetOfTheHeading(){
        return OffsetofTheHeading;
    }

    //TODO:
    // Create getter method/function
}
