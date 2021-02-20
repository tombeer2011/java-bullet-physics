package com.bulletphysics.collision.narrowphase;

import javax.vecmath.Vector3;

public class PointCollector extends DiscreteCollisionDetectorInterface.Result {
    public final Vector3 normalOnBInWorld = new Vector3();
    public final Vector3 pointInWorld = new Vector3();
    public float distance = 1e30f;
    public boolean hasResult = false;

    public void addContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth) {
        if (depth < distance) {
            hasResult = true;
            this.normalOnBInWorld.set(normalOnBInWorld);
            this.pointInWorld.set(pointInWorld);
            distance = depth;
        }
    }
}
