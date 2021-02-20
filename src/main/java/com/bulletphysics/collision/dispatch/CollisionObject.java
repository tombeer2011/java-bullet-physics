package com.bulletphysics.collision.dispatch;

import com.bulletphysics.Transform;
import com.bulletphysics.collision.broadphase.BroadPhaseProxy;
import com.bulletphysics.collision.shapes.CollisionShape;

import javax.vecmath.Vector3;

public class CollisionObject {
    public static final int ACTIVE_TAG = 1;
    public static final int ISLAND_SLEEPING = 2;
    public static final int WANTS_DEACTIVATION = 3;
    public static final int DISABLE_DEACTIVATION = 4;
    public static final int DISABLE_SIMULATION = 5;
    protected final Transform worldTransform = new Transform();
    protected final Transform interpolationWorldTransform = new Transform();
    protected final Vector3 interpolationLinearVelocity = new Vector3();
    protected final Vector3 interpolationAngularVelocity = new Vector3();
    protected BroadPhaseProxy broadphaseHandle;
    protected CollisionShape collisionShape;
    protected CollisionShape rootCollisionShape;
    protected int collisionFlags;
    protected int islandTag1;
    protected int companionId;
    protected int activationState1;
    protected float deactivationTime;
    protected float friction;
    protected float restitution;
    protected CollisionObjectType internalType = CollisionObjectType.COLLISION_OBJECT;
    protected float hitFraction;
    protected float ccdSweptSphereRadius;
    protected float ccdMotionThreshold = 0f;
    protected boolean checkCollideWith;

    public CollisionObject() {
        collisionFlags = CollisionFlags.STATIC_OBJECT;
        islandTag1 = -1;
        companionId = -1;
        activationState1 = 1;
        friction = 0.5f;
        hitFraction = 1f;
    }

    public boolean checkCollideWithOverride() {
        return true;
    }

    public boolean mergesSimulationIslands() {
        return (collisionFlags & (CollisionFlags.STATIC_OBJECT | CollisionFlags.KINEMATIC_OBJECT | CollisionFlags.NO_CONTACT_RESPONSE)) == 0;
    }

    public boolean isStaticObject() {
        return (collisionFlags & CollisionFlags.STATIC_OBJECT) != 0;
    }

    public boolean isKinematicObject() {
        return (collisionFlags & CollisionFlags.KINEMATIC_OBJECT) != 0;
    }

    public boolean isStaticOrKinematicObject() {
        return (collisionFlags & (CollisionFlags.KINEMATIC_OBJECT | CollisionFlags.STATIC_OBJECT)) != 0;
    }

    public boolean hasContactResponse() {
        return (collisionFlags & CollisionFlags.NO_CONTACT_RESPONSE) == 0;
    }

    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    public void setCollisionShape(CollisionShape collisionShape) {
        this.collisionShape = collisionShape;
        rootCollisionShape = collisionShape;
    }

    public int getActivationState() {
        return activationState1;
    }

    public void setActivationState(int newState) {
        if (activationState1 != DISABLE_DEACTIVATION && activationState1 != DISABLE_SIMULATION) {
            activationState1 = newState;
        }
    }

    public void activate() {
        activate(false);
    }

    public void activate(boolean forceActivation) {
        if (forceActivation || (collisionFlags & (CollisionFlags.STATIC_OBJECT | CollisionFlags.KINEMATIC_OBJECT)) == 0) {
            setActivationState(ACTIVE_TAG);
            deactivationTime = 0f;
        }
    }

    public boolean isActive() {
        return getActivationState() != ISLAND_SLEEPING && getActivationState() != DISABLE_SIMULATION;
    }

    public float getRestitution() {
        return restitution;
    }

    public float getFriction() {
        return friction;
    }

    public CollisionObjectType getInternalType() {
        return internalType;
    }

    public Transform getWorldTransform(Transform out) {
        out.set(worldTransform);
        return out;
    }

    public void setWorldTransform(Transform worldTransform) {
        this.worldTransform.set(worldTransform);
    }

    public BroadPhaseProxy getBroadphaseHandle() {
        return broadphaseHandle;
    }

    public void setBroadphaseHandle(BroadPhaseProxy broadphaseHandle) {
        this.broadphaseHandle = broadphaseHandle;
    }

    public Transform getInterpolationWorldTransform(Transform out) {
        out.set(interpolationWorldTransform);
        return out;
    }

    public void setInterpolationWorldTransform(Transform interpolationWorldTransform) {
        this.interpolationWorldTransform.set(interpolationWorldTransform);
    }

    public Vector3 getInterpolationLinearVelocity(Vector3 out) {
        out.set(interpolationLinearVelocity);
        return out;
    }

    public Vector3 getInterpolationAngularVelocity(Vector3 out) {
        out.set(interpolationAngularVelocity);
        return out;
    }

    public int getIslandTag() {
        return islandTag1;
    }

    public void setIslandTag(int islandTag) {
        islandTag1 = islandTag;
    }

    public int getCompanionId() {
        return companionId;
    }

    public void setCompanionId(int companionId) {
        this.companionId = companionId;
    }

    public float getHitFraction() {
        return hitFraction;
    }

    public void setHitFraction(float hitFraction) {
        this.hitFraction = hitFraction;
    }

    public float getCcdSweptSphereRadius() {
        return ccdSweptSphereRadius;
    }

    public void setCcdSweptSphereRadius(float ccdSweptSphereRadius) {
        this.ccdSweptSphereRadius = ccdSweptSphereRadius;
    }

    public float getCcdSquareMotionThreshold() {
        return ccdMotionThreshold * ccdMotionThreshold;
    }

    public void setCcdMotionThreshold(float ccdMotionThreshold) {
        this.ccdMotionThreshold = ccdMotionThreshold;
    }

    public boolean checkCollideWith() {
        if (checkCollideWith) {
            return checkCollideWithOverride();
        }
        return true;
    }
}
