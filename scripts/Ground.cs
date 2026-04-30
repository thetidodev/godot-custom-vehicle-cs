using Godot;
using System;

public partial class Ground : Node3D
{
    [Export] public Color skidColor = new("#000000");
    [Export] public float grip = 1f;
    [Export] public float skidMarkSensitivity = 1f;

}
