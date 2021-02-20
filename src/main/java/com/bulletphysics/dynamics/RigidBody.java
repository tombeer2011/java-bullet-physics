package com.bulletphysics.dynamics;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.MotionState;
import com.bulletphysics.Transform;
import com.bulletphysics.collision.broadphase.BroadPhaseProxy;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionObjectType;

import javax.vecmath.Matrix3;
import javax.vecmath.Vector3;

public class RigidBody extends CollisionObject {
    private static final float MAX_ANGVEL = BulletGlobals.HALF_PI;
    private final Matrix3 invInertiaTensorWorld = new Matrix3();
    private final Vector3 linearVelocity = new Vector3();
    private final Vector3 angularVelocity = new Vector3();
    private float inverseMass;
    private float angularFactor;
    private final Vector3 gravity = new Vector3();
    private final Vector3 invInertiaLocal = new Vector3();
    private final Vector3 totalForce = new Vector3();
    private final Vector3 totalTorque = new Vector3();
    private float linearDamping;
    private float angularDamping;
    private boolean additionalDamping;
    private float additionalDampingFactor;
    private float additionalLinearDampingThresholdSqr;
    private float additionalAngularDampingThresholdSqr;
    private float linearSleepingThreshold;
    private float angularSleepingThreshold;
    private MotionState optionalMotionState;
    public int contactSolverType;
    public int frictionSolverType;
    private static int uniqueId = 0;
    public int debugBodyId;

    public RigidBody(RigidBodyConstructionInfo constructionInfo) {
        setupRigidBody(constructionInfo);
    }

    private void setupRigidBody(RigidBodyConstructionInfo constructionInfo) {
        internalType = CollisionObjectType.RIGID_BODY;
        linearVelocity.set(0f, 0f, 0f);
        angularVelocity.set(0f, 0f, 0f);
        angularFactor = 1f;
        gravity.set(0f, 0f, 0f);
        totalForce.set(0f, 0f, 0f);
        totalTorque.set(0f, 0f, 0f);
        linearDamping = 0f;
        angularDamping = 0.5f;
        linearSleepingThreshold = constructionInfo.linearSleepingThreshold;
        angularSleepingThreshold = constructionInfo.angularSleepingThreshold;
        optionalMotionState = constructionInfo.motionState;
        contactSolverType = 0;
        frictionSolverType = 0;
        additionalDamping = constructionInfo.additionalDamping;
        additionalDampingFactor = constructionInfo.additionalDampingFactor;
        additionalLinearDampingThresholdSqr = constructionInfo.additionalLinearDampingThresholdSqr;
        additionalAngularDampingThresholdSqr = constructionInfo.additionalAngularDampingThresholdSqr;
        if (optionalMotionState != null) {
            optionalMotionState.getWorldTransform(worldTransform);
        } else {
            worldTransform.set(constructionInfo.startWorldTransform);
        }
        interpolationWorldTransform.set(worldTransform);
        interpolationLinearVelocity.set(0f, 0f, 0f);
        interpolationAngularVelocity.set(0f, 0f, 0f);
        friction = constructionInfo.friction;
        restitution = constructionInfo.restitution;
        setCollisionShape(constructionInfo.collisionShape);
        debugBodyId = uniqueId++;
        setMassProps(constructionInfo.mass, constructionInfo.localInertia);
        setDamping(constructionInfo.linearDamping, constructionInfo.angularDamping);
        updateInertiaTensor();
    }

    public void proceedToTransform(Transform newTrans) {
        setCenterOfMassTransform(newTrans);
    }

    public static RigidBody upcast(CollisionObject colObj) {
        if (colObj.getInternalType() == CollisionObjectType.RIGID_BODY) {
            return (RigidBody) colObj;
        }
        return null;
    }

    public void predictIntegratedTransform(float timeStep, Transform predictedTransform) {
        Transform.integrateTransform(worldTransform, linearVelocity, angularVelocity, timeStep, predictedTransform);
    }

    public void saveKinematicState(float timeStep) {
        if (timeStep != 0f) {
            if (getMotionState() != null) {
                getMotionState().getWorldTransform(worldTransform);
            }
            Transform.calculateVelocity(interpolationWorldTransform, worldTransform, timeStep, linearVelocity, angularVelocity);
            interpolationLinearVelocity.set(linearVelocity);
            interpolationAngularVelocity.set(angularVelocity);
            interpolationWorldTransform.set(worldTransform);
        }
    }

    public void applyGravity() {
        if (isStaticOrKinematicObject()) {
            return;
        }
        applyCentralForce(gravity);
    }

    public void setGravity(Vector3 acceleration) {
        if (inverseMass != 0f) {
            gravity.set(acceleration).mul(1f / inverseMass);
        }
    }

    public void setDamping(float lin_damping, float ang_damping) {
        linearDamping = lin_damping < 0f ? 0f : Math.min(1f, lin_damping);
        angularDamping = ang_damping < 0f ? 0f : Math.min(1f, ang_damping);
    }

    public void applyDamping(float timeStep) {
        linearVelocity.mul((float) Math.pow(1f - linearDamping, timeStep));
        angularVelocity.mul((float) Math.pow(1f - angularDamping, timeStep));
        if (additionalDamping) {
            if (angularVelocity.lengthSquared() < additionalAngularDampingThresholdSqr && linearVelocity.lengthSquared() < additionalLinearDampingThresholdSqr) {
                angularVelocity.mul(additionalDampingFactor);
                linearVelocity.mul(additionalDampingFactor);
            }
            var speed = linearVelocity.length();
            if (speed < linearDamping) {
                var dampVel = 0.005f;
                if (speed > dampVel) {
                    var dir = new Vector3(linearVelocity);
                    dir.normalize();
                    dir.mul(dampVel);
                    linearVelocity.sub(dir);
                } else {
                    linearVelocity.set(0f, 0f, 0f);
                }
            }
            var angSpeed = angularVelocity.length();
            if (angSpeed < angularDamping) {
                var angDampVel = 0.005f;
                if (angSpeed > angDampVel) {
                    var dir = new Vector3(angularVelocity);
                    dir.normalize();
                    dir.mul(angDampVel);
                    angularVelocity.sub(dir);
                } else {
                    angularVelocity.set(0f, 0f, 0f);
                }
            }
        }
    }

    public void setMassProps(float mass, Vector3 inertia) {
        if (mass == 0f) {
            collisionFlags |= CollisionFlags.STATIC_OBJECT;
            inverseMass = 0f;
        } else {
            collisionFlags &= ~CollisionFlags.STATIC_OBJECT;
            inverseMass = 1f / mass;
        }
        invInertiaLocal.set(inertia.x != 0f ? 1f / inertia.x : 0f, inertia.y != 0f ? 1f / inertia.y : 0f, inertia.z != 0f ? 1f / inertia.z : 0f);
    }

    public float getInvMass() {
        return inverseMass;
    }

    public Matrix3 getInvInertiaTensorWorld(Matrix3 out) {
        out.set(invInertiaTensorWorld);
        return out;
    }

    public void integrateVelocities(float step) {
        if (isStaticOrKinematicObject()) {
            return;
        }
        linearVelocity.scaleAdd(inverseMass * step, totalForce, linearVelocity);
        var tmp = new Vector3(totalTorque);
        tmp.transform(invInertiaTensorWorld);
        angularVelocity.scaleAdd(step, tmp, angularVelocity);
        var angvel = angularVelocity.length();
        if (angvel * step > MAX_ANGVEL) {
            angularVelocity.mul(MAX_ANGVEL / step / angvel);
        }
    }

    public void setCenterOfMassTransform(Transform xform) {
        if (isStaticOrKinematicObject()) {
            interpolationWorldTransform.set(worldTransform);
        } else {
            interpolationWorldTransform.set(xform);
        }
        getLinearVelocity(interpolationLinearVelocity);
        getAngularVelocity(interpolationAngularVelocity);
        worldTransform.set(xform);
        updateInertiaTensor();
    }

    public void applyCentralForce(Vector3 force) {
        totalForce.add(force);
    }

    public void clearForces() {
        totalForce.set(0f, 0f, 0f);
        totalTorque.set(0f, 0f, 0f);
    }

    public void updateInertiaTensor() {
        var mat1 = new Matrix3();
        mat1.scale(worldTransform.basis, invInertiaLocal);
        var mat2 = new Matrix3(worldTransform.basis);
        mat2.transpose();
        invInertiaTensorWorld.mul(mat1, mat2);
    }

    public Vector3 getLinearVelocity(Vector3 out) {
        out.set(linearVelocity);
        return out;
    }

    public Vector3 getAngularVelocity(Vector3 out) {
        out.set(angularVelocity);
        return out;
    }

    public void setLinearVelocity(Vector3 lin_vel) {

        linearVelocity.set(lin_vel);
    }

    public void setAngularVelocity(Vector3 ang_vel) {

        angularVelocity.set(ang_vel);
    }

    public Vector3 getVelocityInLocalPoint(Vector3 rel_pos, Vector3 out) {
        out.cross(angularVelocity, rel_pos);
        out.add(linearVelocity);
        return out;
    }

    public void updateDeactivation(float timeStep) {
        if (getActivationState() == ISLAND_SLEEPING || getActivationState() == DISABLE_DEACTIVATION) {
            return;
        }
        if (getLinearVelocity(new Vector3()).lengthSquared() < linearSleepingThreshold * linearSleepingThreshold && getAngularVelocity(new Vector3()).lengthSquared() < angularSleepingThreshold * angularSleepingThreshold) {
            deactivationTime += timeStep;
        } else {
            deactivationTime = 0f;
            setActivationState(0);
        }
    }

    public boolean wantsSleeping() {
        if (getActivationState() == DISABLE_DEACTIVATION) {
            return false;
        }
        if (BulletGlobals.DISABLE_DEACTIVATION || BulletGlobals.DEACTIVATION_TIME == 0f) {
            return false;
        }
        if (getActivationState() == ISLAND_SLEEPING || getActivationState() == WANTS_DEACTIVATION) {
            return true;
        }
        return deactivationTime > BulletGlobals.DEACTIVATION_TIME;
    }

    public BroadPhaseProxy getBroadphaseProxy() {
        return broadphaseHandle;
    }

    public MotionState getMotionState() {
        return optionalMotionState;
    }

    public float getAngularFactor() {
        return angularFactor;
    }
}
