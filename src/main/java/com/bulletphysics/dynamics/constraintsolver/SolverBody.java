package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.Transform;
import com.bulletphysics.dynamics.RigidBody;

import javax.vecmath.Vector3;

public class SolverBody {
    public final Vector3 angularVelocity = new Vector3();
    public float angularFactor;
    public float invMass;
    public float friction;
    public RigidBody originalBody;
    public final Vector3 linearVelocity = new Vector3();
    public final Vector3 centerOfMassPosition = new Vector3();
    public final Vector3 pushVelocity = new Vector3();
    public final Vector3 turnVelocity = new Vector3();

    public void internalApplyImpulse(Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude) {
        if (invMass != 0f) {
            linearVelocity.scaleAdd(impulseMagnitude, linearComponent, linearVelocity);
            angularVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, angularVelocity);
        }
    }

    public void internalApplyPushImpulse(Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude) {
        if (invMass != 0f) {
            pushVelocity.scaleAdd(impulseMagnitude, linearComponent, pushVelocity);
            turnVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, turnVelocity);
        }
    }

    public void writebackVelocity() {
        if (invMass != 0f) {
            originalBody.setLinearVelocity(linearVelocity);
            originalBody.setAngularVelocity(angularVelocity);
        }
    }

    public void writebackVelocity(float timeStep) {
        if (invMass != 0f) {
            originalBody.setLinearVelocity(linearVelocity);
            originalBody.setAngularVelocity(angularVelocity);
            var newTransform = new Transform();
            var curTrans = originalBody.getWorldTransform(new Transform());
            Transform.integrateTransform(curTrans, pushVelocity, turnVelocity, timeStep, newTransform);
            originalBody.setWorldTransform(newTransform);
        }
    }
}
