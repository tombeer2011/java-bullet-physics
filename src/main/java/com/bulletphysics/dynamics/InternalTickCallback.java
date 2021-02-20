package com.bulletphysics.dynamics;

public abstract class InternalTickCallback {
    public abstract void internalTick(DynamicsWorld world, float timeStep);
}
