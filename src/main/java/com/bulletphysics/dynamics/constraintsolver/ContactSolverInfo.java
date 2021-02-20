package com.bulletphysics.dynamics.constraintsolver;

public class ContactSolverInfo {
    public float timeStep;
    public int numIterations = 10;
    public float erp = 0.2f;
    public final float erp2 = 0.1f;
    public final boolean splitImpulse = false;
    public final float splitImpulsePenetrationThreshold = -0.02f;
    public final float linearSlop = 0f;
}
