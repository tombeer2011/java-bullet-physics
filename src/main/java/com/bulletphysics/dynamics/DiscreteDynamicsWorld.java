package com.bulletphysics.dynamics;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.Transform;
import com.bulletphysics.collision.broadphase.*;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.SimulationIslandManager;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.ContactSolverInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;

import javax.vecmath.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class DiscreteDynamicsWorld extends DynamicsWorld {
    protected ConstraintSolver constraintSolver;
    protected final SimulationIslandManager islandManager;
    protected final Vector3 gravity = new Vector3(0f, -10f, 0f);
    protected float localTime = 1f / 60f;
    protected final boolean ownsIslandManager;
    protected boolean ownsConstraintSolver;
    protected final ArrayList<ActionInterface> actions = new ArrayList<>();

    public DiscreteDynamicsWorld(Dispatcher dispatcher, BroadPhaseInterface pairCache, ConstraintSolver constraintSolver) {
        super(dispatcher, pairCache);
        this.constraintSolver = constraintSolver;
        if (this.constraintSolver == null) {
            this.constraintSolver = new SequentialImpulseConstraintSolver();
            ownsConstraintSolver = true;
        } else {
            ownsConstraintSolver = false;
        }
        islandManager = new SimulationIslandManager();
        ownsIslandManager = true;
    }

    protected void saveKinematicState(float timeStep) {
        for (CollisionObject colObj : collisionObjects) {
            var body = RigidBody.upcast(colObj);
            if (body != null) {
                if (body.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                    if (body.isKinematicObject()) {
                        body.saveKinematicState(timeStep);
                    }
                }
            }
        }
    }

    @Override
    public void clearForces() {
        for (CollisionObject colObj : collisionObjects) {
            var body = RigidBody.upcast(colObj);
            if (body != null) {
                body.clearForces();
            }
        }
    }

    public void applyGravity() {
        for (CollisionObject colObj : collisionObjects) {
            var body = RigidBody.upcast(colObj);
            if (body != null && body.isActive()) {
                body.applyGravity();
            }
        }
    }

    protected void synchronizeMotionStates() {
        var interpolatedTransform = new Transform();
        var tmpTrans = new Transform();
        var tmpLinVel = new Vector3();
        var tmpAngVel = new Vector3();
        for (CollisionObject colObj : collisionObjects) {
            var body = RigidBody.upcast(colObj);
            if (body != null && body.getMotionState() != null && !body.isStaticOrKinematicObject()) {
                Transform.integrateTransform(body.getInterpolationWorldTransform(tmpTrans), body.getInterpolationLinearVelocity(tmpLinVel), body.getInterpolationAngularVelocity(tmpAngVel), localTime * body.getHitFraction(), interpolatedTransform);
                body.getMotionState().setWorldTransform(interpolatedTransform);
            }
        }
    }

    @Override
    public void stepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep) {
        var numSimulationSubSteps = 0;
        if (maxSubSteps != 0) {
            localTime += timeStep;
            if (localTime >= fixedTimeStep) {
                numSimulationSubSteps = (int) (localTime / fixedTimeStep);
                localTime -= numSimulationSubSteps * fixedTimeStep;
            }
        } else {
            fixedTimeStep = timeStep;
            localTime = timeStep;
            if (Math.abs(timeStep) < BulletGlobals.EPSILON) {
                numSimulationSubSteps = 0;
                maxSubSteps = 0;
            } else {
                numSimulationSubSteps = 1;
                maxSubSteps = 1;
            }
        }
        if (numSimulationSubSteps != 0) {
            saveKinematicState(fixedTimeStep);
            applyGravity();
            var clampedSimulationSteps = Math.min(numSimulationSubSteps, maxSubSteps);
            for (var i = 0; i < clampedSimulationSteps; i++) {
                internalSingleStepSimulation(fixedTimeStep);
                synchronizeMotionStates();
            }
        }
        synchronizeMotionStates();
        clearForces();
    }

    protected void internalSingleStepSimulation(float timeStep) {
        predictUnconstraintMotion(timeStep);
        var dispatchInfo = getDispatchInfo();
        dispatchInfo.timeStep = timeStep;
        dispatchInfo.stepCount = 0;
        performDiscreteCollisionDetection();
        calculateSimulationIslands();
        getSolverInfo().timeStep = timeStep;
        solveConstraints(getSolverInfo());
        integrateTransforms(timeStep);
        updateActions(timeStep);
        updateActivationState(timeStep);
        if (internalTickCallback != null) {
            internalTickCallback.internalTick(this, timeStep);
        }
    }

    @Override
    public void setGravity(Vector3 gravity) {
        this.gravity.set(gravity);
        for (CollisionObject colObj : collisionObjects) {
            var body = RigidBody.upcast(colObj);
            if (body != null) {
                body.setGravity(gravity);
            }
        }
    }

    @Override
    public void addRigidBody(RigidBody body) {
        if (!body.isStaticOrKinematicObject()) {
            body.setGravity(gravity);
        }
        if (body.getCollisionShape() != null) {
            var isDynamic = !(body.isStaticObject() || body.isKinematicObject());
            var collisionFilterGroup = isDynamic ? CollisionFilterGroups.DEFAULT_FILTER : CollisionFilterGroups.STATIC_FILTER;
            var collisionFilterMask = isDynamic ? CollisionFilterGroups.ALL_FILTER : (short) (CollisionFilterGroups.ALL_FILTER ^ CollisionFilterGroups.STATIC_FILTER);
            addCollisionObject(body, collisionFilterGroup, collisionFilterMask);
        }
    }

    public void updateActions(float timeStep) {
        actions.forEach(action -> action.updateAction(this, timeStep));
    }

    protected void updateActivationState(float timeStep) {
        var tmp = new Vector3();
        collisionObjects.stream().map(RigidBody::upcast).filter(Objects::nonNull).forEach(body -> {
            body.updateDeactivation(timeStep);
            if (body.wantsSleeping()) {
                if (body.isStaticOrKinematicObject()) {
                    body.setActivationState(CollisionObject.ISLAND_SLEEPING);
                } else {
                    if (body.getActivationState() == CollisionObject.ACTIVE_TAG) {
                        body.setActivationState(CollisionObject.WANTS_DEACTIVATION);
                    }
                    if (body.getActivationState() == CollisionObject.ISLAND_SLEEPING) {
                        tmp.set(0f, 0f, 0f);
                        body.setAngularVelocity(tmp);
                        body.setLinearVelocity(tmp);
                    }
                }
            } else {
                if (body.getActivationState() != CollisionObject.DISABLE_DEACTIVATION) {
                    body.setActivationState(CollisionObject.ACTIVE_TAG);
                }
            }
        });
    }

    private static class InPlaceSolverIslandCallback extends SimulationIslandManager.IslandCallback {
        public ContactSolverInfo solverInfo;
        public ConstraintSolver solver;
        public Dispatcher dispatcher;

        public void init(ContactSolverInfo solverInfo, ConstraintSolver solver, Dispatcher dispatcher) {
            this.solverInfo = solverInfo;
            this.solver = solver;
            this.dispatcher = dispatcher;
        }

        public void processIsland(List<CollisionObject> bodies, int numBodies, List<PersistentManifold> manifolds, int manifolds_offset, int numManifolds, int islandId) {
            if (islandId < 0 || numManifolds > 0) {
                solver.solveGroup(bodies, numBodies, manifolds, manifolds_offset, numManifolds, solverInfo);
            }
        }
    }

    private final InPlaceSolverIslandCallback solverCallback = new InPlaceSolverIslandCallback();

    protected void solveConstraints(ContactSolverInfo solverInfo) {
        solverCallback.init(solverInfo, constraintSolver, dispatcher1);
        getCollisionWorld().getNumCollisionObjects();
        getCollisionWorld().getDispatcher().getNumManifolds();
        islandManager.buildAndProcessIslands(getCollisionWorld().getDispatcher(), getCollisionWorld().getCollisionObjectArray(), solverCallback);
    }

    protected void calculateSimulationIslands() {
        getSimulationIslandManager().updateActivationState(getCollisionWorld());
        getSimulationIslandManager().storeIslandActivationState(getCollisionWorld());
    }

    protected void integrateTransforms(float timeStep) {
        var tmp = new Vector3();
        var tmpTrans = new Transform();
        var predictedTrans = new Transform();
        for (CollisionObject colObj : collisionObjects) {
            var body = RigidBody.upcast(colObj);
            if (body != null) {
                body.setHitFraction(1f);
                if (body.isActive() && !body.isStaticOrKinematicObject()) {
                    body.predictIntegratedTransform(timeStep, predictedTrans);
                    tmp.set(predictedTrans.origin).sub(body.getWorldTransform(tmpTrans).origin);
                    var squareMotion = tmp.lengthSquared();
                    if (body.getCcdSquareMotionThreshold() != 0f && body.getCcdSquareMotionThreshold() < squareMotion) {
                        if (body.getCollisionShape().isConvex()) {
                            var sweepResults = new ClosestNotMeConvexResultCallback(body, body.getWorldTransform(tmpTrans).origin, predictedTrans.origin, getBroadphase().getOverlappingPairCache(), getDispatcher());
                            var tmpSphere = new SphereShape(body.getCcdSweptSphereRadius());
                            sweepResults.collisionFilterGroup = body.getBroadphaseProxy().collisionFilterGroup;
                            sweepResults.collisionFilterMask = body.getBroadphaseProxy().collisionFilterMask;
                            convexSweepTest(tmpSphere, body.getWorldTransform(tmpTrans), predictedTrans, sweepResults);
                            if (sweepResults.hasHit() && sweepResults.closestHitFraction > 0.0001f) {
                                body.setHitFraction(sweepResults.closestHitFraction);
                                body.predictIntegratedTransform(timeStep * body.getHitFraction(), predictedTrans);
                                body.setHitFraction(0f);
                            }
                        }
                    }
                    body.proceedToTransform(predictedTrans);
                }
            }
        }
    }

    protected void predictUnconstraintMotion(float timeStep) {
        var tmpTrans = new Transform();
        for (CollisionObject colObj : collisionObjects) {
            var body = RigidBody.upcast(colObj);
            if (body != null) {
                if (!body.isStaticOrKinematicObject()) {
                    if (body.isActive()) {
                        body.integrateVelocities(timeStep);
                        body.applyDamping(timeStep);
                        body.predictIntegratedTransform(timeStep, body.getInterpolationWorldTransform(tmpTrans));
                    }
                }
            }
        }
    }

    public SimulationIslandManager getSimulationIslandManager() {
        return islandManager;
    }

    public CollisionWorld getCollisionWorld() {
        return this;
    }

    private static class ClosestNotMeConvexResultCallback extends ClosestConvexResultCallback {
        private final CollisionObject me;
        private final OverlappingPairCache pairCache;
        private final Dispatcher dispatcher;

        public ClosestNotMeConvexResultCallback(CollisionObject me, Vector3 fromA, Vector3 toA, OverlappingPairCache pairCache, Dispatcher dispatcher) {
            super(fromA, toA);
            this.me = me;
            this.pairCache = pairCache;
            this.dispatcher = dispatcher;
        }

        @Override
        public float addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace) {
            if (convexResult.hitCollisionObject == me) {
                return 1f;
            }
            Vector3 linVelA = new Vector3(), linVelB = new Vector3();
            linVelA.set(convexToWorld).sub(convexFromWorld);
            linVelB.set(0f, 0f, 0f);
            var relativeVelocity = new Vector3();
            relativeVelocity.set(linVelA).sub(linVelB);
            var allowedPenetration = 0f;
            if (convexResult.hitNormalLocal.dot(relativeVelocity) >= -allowedPenetration) {
                return 1f;
            }
            return super.addSingleResult(convexResult, normalInWorldSpace);
        }

        @Override
        public boolean needsCollision(BroadPhaseProxy proxy0) {
            if (proxy0.clientObject == me) {
                return false;
            }
            if (!super.needsCollision(proxy0)) {
                return false;
            }
            var otherObj = (CollisionObject) proxy0.clientObject;
            if (dispatcher.needsResponse(me, otherObj)) {
                var manifoldArray = new ArrayList<PersistentManifold>();
                var collisionPair = pairCache.findPair(me.getBroadphaseHandle(), proxy0);
                if (collisionPair != null) {
                    if (collisionPair.algorithm != null) {
                        collisionPair.algorithm.getAllContactManifolds(manifoldArray);
                        return manifoldArray.stream().noneMatch(manifold -> manifold.getNumContacts() > 0);
                    }
                }
            }
            return true;
        }
    }
}
