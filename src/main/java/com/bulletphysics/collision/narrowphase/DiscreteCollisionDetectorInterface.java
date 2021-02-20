package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.Transform;

import javax.vecmath.Vector3;

public abstract class DiscreteCollisionDetectorInterface {
    public static abstract class Result {
        public abstract void addContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth);
    }

    public static class ClosestPointInput {
        public final Transform transformA = new Transform();
        public final Transform transformB = new Transform();
        public float maximumDistanceSquared = Float.MAX_VALUE;
    }
}
