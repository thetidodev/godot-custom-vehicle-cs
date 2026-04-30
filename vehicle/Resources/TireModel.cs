using Godot;

[GlobalClass]
public partial class TireModel : Resource
{
    [Export] string model_name = "Default (Linear)";

    [Export] public float PeakY = 1; // pacejka = 1 # default = 0.748 # (float,0.001,1)
    [Export] public float PeakX = 0.936f; // pacejka = 0.936 # default = 0.748 # (float,0.001,1)
    [Export] public float AspectRatio = 0.228f; // pacejka = 0.228 # default = 1 # (float,0.001,1)
    [Export] public float ShapeX = 1; // pacejka = 0.855 # default = 1 # (float,0,1)
    [Export] public float ShapeY = 1; // pacejka = 0.975 # default = 1 # (float,0,1)
    [Export] public float Linear = 0; // (float,0,1)
    [Export] public float Friction = 1;

}
