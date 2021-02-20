package com.bulletphysics.dynamics;

import com.bulletphysics.collision.broadphase.BroadPhaseInterface;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.dynamics.constraintsolver.ContactSolverInfo;

import javax.vecmath.Vector3;

public abstract class DynamicsWorld extends CollisionWorld {
    protected InternalTickCallback internalTickCallback;
    protected final ContactSolverInfo solverInfo = new ContactSolverInfo();

    public DynamicsWorld(Dispatcher dispatcher, BroadPhaseInterface broadphasePairCache) {
        super(dispatcher, broadphasePairCache);
    }

    public final void stepSimulation(float timeStep) {
        stepSimulation(timeStep, 1, 1f / 60f);
    }

    public final void stepSimulation(float timeStep, int maxSubSteps) {
        stepSimulation(timeStep, maxSubSteps, 1f / 60f);
    }

    public abstract void stepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep);

    public abstract void setGravity(Vector3 gravity);

    public abstract void addRigidBody(RigidBody body);

    public abstract void clearForces();

    public ContactSolverInfo getSolverInfo() {
        return solverInfo;
    }
}
