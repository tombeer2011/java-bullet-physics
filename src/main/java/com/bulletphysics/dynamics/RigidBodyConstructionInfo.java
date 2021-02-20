package com.bulletphysics.dynamics;

import com.bulletphysics.MotionState;
import com.bulletphysics.Transform;
import com.bulletphysics.collision.shapes.CollisionShape;

import javax.vecmath.Vector3;

public class RigidBodyConstructionInfo {
    public final float mass;
    public final MotionState motionState;
    public final Transform startWorldTransform = new Transform();
    public final CollisionShape collisionShape;
    public final Vector3 localInertia = new Vector3();
    public final float linearDamping = 0f;
    public final float angularDamping = 0f;
    public final float friction = 0.5f;
    public final float restitution = 0f;
    public final float linearSleepingThreshold = 0.8f;
    public final float angularSleepingThreshold = 1.0f;
    public final boolean additionalDamping = false;
    public final float additionalDampingFactor = 0.005f;
    public final float additionalLinearDampingThresholdSqr = 0.01f;
    public final float additionalAngularDampingThresholdSqr = 0.01f;

    public RigidBodyConstructionInfo(float mass, MotionState motionState, CollisionShape collisionShape, Vector3 localInertia) {
        this.mass = mass;
        this.motionState = motionState;
        this.collisionShape = collisionShape;
        this.localInertia.set(localInertia);
        startWorldTransform.identity();
    }
}
