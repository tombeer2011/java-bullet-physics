package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.Transform;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.dynamics.RigidBody;

import javax.vecmath.Matrix3;
import javax.vecmath.Vector3;
import java.util.ArrayList;
import java.util.List;

public class SequentialImpulseConstraintSolver implements ConstraintSolver {
    private final List<SolverBody> tmpSolverBodyPool = new ArrayList<>();
    private final List<SolverConstraint> tmpSolverConstraintPool = new ArrayList<>();
    private final List<SolverConstraint> tmpSolverFrictionConstraintPool = new ArrayList<>();
    private final int[] orderTmpConstraintPool = new int[4096];
    private final int[] orderFrictionConstraintPool = new int[4096];

    private void initSolverBody(SolverBody solverBody, CollisionObject collisionObject) {
        var rb = RigidBody.upcast(collisionObject);
        if (rb != null) {
            rb.getAngularVelocity(solverBody.angularVelocity);
            solverBody.centerOfMassPosition.set(collisionObject.getWorldTransform(new Transform()).origin);
            solverBody.friction = collisionObject.getFriction();
            solverBody.invMass = rb.getInvMass();
            rb.getLinearVelocity(solverBody.linearVelocity);
            solverBody.originalBody = rb;
            solverBody.angularFactor = rb.getAngularFactor();
        } else {
            solverBody.angularVelocity.set(0f, 0f, 0f);
            solverBody.centerOfMassPosition.set(collisionObject.getWorldTransform(new Transform()).origin);
            solverBody.friction = collisionObject.getFriction();
            solverBody.invMass = 0f;
            solverBody.linearVelocity.set(0f, 0f, 0f);
            solverBody.originalBody = null;
            solverBody.angularFactor = 1f;
        }
        solverBody.pushVelocity.set(0f, 0f, 0f);
        solverBody.turnVelocity.set(0f, 0f, 0f);
    }

    private float restitutionCurve(float rel_vel, float restitution) {
        return restitution * -rel_vel;
    }

    private void resolveSplitPenetrationImpulseCacheFriendly(SolverBody body1, SolverBody body2, SolverConstraint contactConstraint, ContactSolverInfo solverInfo) {
        if (contactConstraint.penetration < solverInfo.splitImpulsePenetrationThreshold) {
            float normalImpulse;
            float rel_vel;
            var vel1Dotn = contactConstraint.contactNormal.dot(body1.pushVelocity) + contactConstraint.relpos1CrossNormal.dot(body1.turnVelocity);
            var vel2Dotn = contactConstraint.contactNormal.dot(body2.pushVelocity) + contactConstraint.relpos2CrossNormal.dot(body2.turnVelocity);
            rel_vel = vel1Dotn - vel2Dotn;
            var positionalError = -contactConstraint.penetration * solverInfo.erp2 / solverInfo.timeStep;
            var velocityError = contactConstraint.restitution - rel_vel;
            var penetrationImpulse = positionalError * contactConstraint.jacDiagABInv;
            var velocityImpulse = velocityError * contactConstraint.jacDiagABInv;
            normalImpulse = penetrationImpulse + velocityImpulse;
            var oldNormalImpulse = contactConstraint.appliedPushImpulse;
            var sum = oldNormalImpulse + normalImpulse;
            contactConstraint.appliedPushImpulse = Math.max(0f, sum);
            normalImpulse = contactConstraint.appliedPushImpulse - oldNormalImpulse;
            var tmp = new Vector3();
            tmp.set(contactConstraint.contactNormal).mul(body1.invMass);
            body1.internalApplyPushImpulse(tmp, contactConstraint.angularComponentA, normalImpulse);
            tmp.set(contactConstraint.contactNormal).mul(body2.invMass);
            body2.internalApplyPushImpulse(tmp, contactConstraint.angularComponentB, -normalImpulse);
        }
    }

    private float resolveSingleCollisionCombinedCacheFriendly(SolverBody body1, SolverBody body2, SolverConstraint contactConstraint, ContactSolverInfo solverInfo) {
        float normalImpulse;
        float rel_vel;
        var vel1Dotn = contactConstraint.contactNormal.dot(body1.linearVelocity) + contactConstraint.relpos1CrossNormal.dot(body1.angularVelocity);
        var vel2Dotn = contactConstraint.contactNormal.dot(body2.linearVelocity) + contactConstraint.relpos2CrossNormal.dot(body2.angularVelocity);
        rel_vel = vel1Dotn - vel2Dotn;
        var positionalError = 0.f;
        if (!solverInfo.splitImpulse || contactConstraint.penetration > solverInfo.splitImpulsePenetrationThreshold) {
            positionalError = -contactConstraint.penetration * solverInfo.erp / solverInfo.timeStep;
        }
        var velocityError = contactConstraint.restitution - rel_vel;
        var penetrationImpulse = positionalError * contactConstraint.jacDiagABInv;
        var velocityImpulse = velocityError * contactConstraint.jacDiagABInv;
        normalImpulse = penetrationImpulse + velocityImpulse;
        var oldNormalImpulse = contactConstraint.appliedImpulse;
        var sum = oldNormalImpulse + normalImpulse;
        contactConstraint.appliedImpulse = Math.max(0f, sum);
        normalImpulse = contactConstraint.appliedImpulse - oldNormalImpulse;
        var tmp = new Vector3();
        tmp.set(contactConstraint.contactNormal).mul(body1.invMass);
        body1.internalApplyImpulse(tmp, contactConstraint.angularComponentA, normalImpulse);
        tmp.set(contactConstraint.contactNormal).mul(body2.invMass);
        body2.internalApplyImpulse(tmp, contactConstraint.angularComponentB, -normalImpulse);
        return normalImpulse;
    }

    private float resolveSingleFrictionCacheFriendly(SolverBody body1, SolverBody body2, SolverConstraint contactConstraint, float appliedNormalImpulse) {
        var combinedFriction = contactConstraint.friction;
        var limit = appliedNormalImpulse * combinedFriction;
        if (appliedNormalImpulse > 0f) {
            float rel_vel;
            var vel1Dotn = contactConstraint.contactNormal.dot(body1.linearVelocity) + contactConstraint.relpos1CrossNormal.dot(body1.angularVelocity);
            var vel2Dotn = contactConstraint.contactNormal.dot(body2.linearVelocity) + contactConstraint.relpos2CrossNormal.dot(body2.angularVelocity);
            rel_vel = vel1Dotn - vel2Dotn;
            var j1 = -rel_vel * contactConstraint.jacDiagABInv;
            var oldTangentImpulse = contactConstraint.appliedImpulse;
            contactConstraint.appliedImpulse = oldTangentImpulse + j1;
            if (limit < contactConstraint.appliedImpulse) {
                contactConstraint.appliedImpulse = limit;
            } else {
                if (contactConstraint.appliedImpulse < -limit) {
                    contactConstraint.appliedImpulse = -limit;
                }
            }
            j1 = contactConstraint.appliedImpulse - oldTangentImpulse;
            var tmp = new Vector3();
            tmp.set(contactConstraint.contactNormal).mul(body1.invMass);
            body1.internalApplyImpulse(tmp, contactConstraint.angularComponentA, j1);
            tmp.set(contactConstraint.contactNormal).mul(body2.invMass);
            body2.internalApplyImpulse(tmp, contactConstraint.angularComponentB, -j1);
        }
        return 0f;
    }

    protected void addFrictionConstraint(Vector3 normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, ManifoldPoint cp, Vector3 rel_pos1, Vector3 rel_pos2, CollisionObject colObj0, CollisionObject colObj1, float relaxation) {
        var body0 = RigidBody.upcast(colObj0);
        var body1 = RigidBody.upcast(colObj1);
        var solverConstraint = new SolverConstraint();
        tmpSolverFrictionConstraintPool.add(solverConstraint);
        solverConstraint.contactNormal.set(normalAxis);
        solverConstraint.solverBodyIdA = solverBodyIdA;
        solverConstraint.solverBodyIdB = solverBodyIdB;
        solverConstraint.constraintType = SolverConstraintType.SOLVER_FRICTION_1D;
        solverConstraint.frictionIndex = frictionIndex;
        solverConstraint.friction = cp.combinedFriction;
        solverConstraint.originalContactPoint = null;
        solverConstraint.appliedImpulse = 0f;
        solverConstraint.appliedPushImpulse = 0f;
        solverConstraint.penetration = 0f;
        var ftorqueAxis1 = new Vector3();
        var tmpMat = new Matrix3();
        ftorqueAxis1.cross(rel_pos1, solverConstraint.contactNormal);
        solverConstraint.relpos1CrossNormal.set(ftorqueAxis1);
        if (body0 != null) {
            solverConstraint.angularComponentA.set(ftorqueAxis1);
            solverConstraint.angularComponentA.transform(body0.getInvInertiaTensorWorld(tmpMat));
        } else {
            solverConstraint.angularComponentA.set(0f, 0f, 0f);
        }
        ftorqueAxis1.cross(rel_pos2, solverConstraint.contactNormal);
        solverConstraint.relpos2CrossNormal.set(ftorqueAxis1);
        if (body1 != null) {
            solverConstraint.angularComponentB.set(ftorqueAxis1);
            solverConstraint.angularComponentB.transform(body1.getInvInertiaTensorWorld(tmpMat));
        } else {
            solverConstraint.angularComponentB.set(0f, 0f, 0f);
        }
        var vec = new Vector3();
        var denom0 = 0f;
        var denom1 = 0f;
        if (body0 != null) {
            vec.cross(solverConstraint.angularComponentA, rel_pos1);
            denom0 = body0.getInvMass() + normalAxis.dot(vec);
        }
        if (body1 != null) {
            vec.cross(solverConstraint.angularComponentB, rel_pos2);
            denom1 = body1.getInvMass() + normalAxis.dot(vec);
        }
        solverConstraint.jacDiagABInv = relaxation / (denom0 + denom1);
    }

    public void solveGroupCacheFriendlySetup(List<PersistentManifold> manifoldPtr, int manifold_offset, int numManifolds, int numConstraints, ContactSolverInfo infoGlobal) {
        if (numConstraints + numManifolds == 0) {
            return;
        }
        PersistentManifold manifold;
        CollisionObject colObj0, colObj1;
        var tmpTrans = new Transform();
        int i;
        var rel_pos1 = new Vector3();
        var rel_pos2 = new Vector3();
        var pos1 = new Vector3();
        var pos2 = new Vector3();
        var vel = new Vector3();
        var torqueAxis0 = new Vector3();
        var torqueAxis1 = new Vector3();
        var vel1 = new Vector3();
        var vel2 = new Vector3();
        var vec = new Vector3();
        var tmpMat = new Matrix3();
        for (i = 0; i < numManifolds; i++) {
            manifold = manifoldPtr.get(manifold_offset + i);
            colObj0 = (CollisionObject) manifold.getBody0();
            colObj1 = (CollisionObject) manifold.getBody1();
            var solverBodyIdA = -1;
            var solverBodyIdB = -1;
            if (manifold.getNumContacts() != 0) {
                if (colObj0.getIslandTag() >= 0) {
                    if (colObj0.getCompanionId() >= 0) {
                        solverBodyIdA = colObj0.getCompanionId();
                    } else {
                        solverBodyIdA = tmpSolverBodyPool.size();
                        var solverBody = new SolverBody();
                        tmpSolverBodyPool.add(solverBody);
                        initSolverBody(solverBody, colObj0);
                        colObj0.setCompanionId(solverBodyIdA);
                    }
                } else {
                    solverBodyIdA = tmpSolverBodyPool.size();
                    var solverBody = new SolverBody();
                    tmpSolverBodyPool.add(solverBody);
                    initSolverBody(solverBody, colObj0);
                }
                if (colObj1.getIslandTag() >= 0) {
                    if (colObj1.getCompanionId() >= 0) {
                        solverBodyIdB = colObj1.getCompanionId();
                    } else {
                        solverBodyIdB = tmpSolverBodyPool.size();
                        var solverBody = new SolverBody();
                        tmpSolverBodyPool.add(solverBody);
                        initSolverBody(solverBody, colObj1);
                        colObj1.setCompanionId(solverBodyIdB);
                    }
                } else {
                    solverBodyIdB = tmpSolverBodyPool.size();
                    var solverBody = new SolverBody();
                    tmpSolverBodyPool.add(solverBody);
                    initSolverBody(solverBody, colObj1);
                }
            }
            float relaxation;
            for (var j = 0; j < manifold.getNumContacts(); j++) {
                var cp = manifold.getContactPoint(j);
                if (cp.getDistance() <= 0f) {
                    cp.getPositionWorldOnA(pos1);
                    cp.getPositionWorldOnB(pos2);
                    rel_pos1.set(pos1).sub(colObj0.getWorldTransform(tmpTrans).origin);
                    rel_pos2.set(pos2).sub(colObj1.getWorldTransform(tmpTrans).origin);
                    relaxation = 1f;
                    float rel_vel;
                    var frictionIndex = tmpSolverConstraintPool.size();
                    var solverConstraint = new SolverConstraint();
                    tmpSolverConstraintPool.add(solverConstraint);
                    var rb0 = RigidBody.upcast(colObj0);
                    var rb1 = RigidBody.upcast(colObj1);
                    solverConstraint.solverBodyIdA = solverBodyIdA;
                    solverConstraint.solverBodyIdB = solverBodyIdB;
                    solverConstraint.constraintType = SolverConstraintType.SOLVER_CONTACT_1D;
                    solverConstraint.originalContactPoint = cp;
                    torqueAxis0.cross(rel_pos1, cp.normalWorldOnB);
                    if (rb0 != null) {
                        solverConstraint.angularComponentA.set(torqueAxis0);
                        solverConstraint.angularComponentA.transform(rb0.getInvInertiaTensorWorld(tmpMat));
                    } else {
                        solverConstraint.angularComponentA.set(0f, 0f, 0f);
                    }
                    torqueAxis1.cross(rel_pos2, cp.normalWorldOnB);
                    if (rb1 != null) {
                        solverConstraint.angularComponentB.set(torqueAxis1);
                        solverConstraint.angularComponentB.transform(rb1.getInvInertiaTensorWorld(tmpMat));
                    } else {
                        solverConstraint.angularComponentB.set(0f, 0f, 0f);
                    }
                    var denom0 = 0f;
                    var denom1 = 0f;
                    if (rb0 != null) {
                        vec.cross(solverConstraint.angularComponentA, rel_pos1);
                        denom0 = rb0.getInvMass() + cp.normalWorldOnB.dot(vec);
                    }
                    if (rb1 != null) {
                        vec.cross(solverConstraint.angularComponentB, rel_pos2);
                        denom1 = rb1.getInvMass() + cp.normalWorldOnB.dot(vec);
                    }
                    solverConstraint.jacDiagABInv = relaxation / (denom0 + denom1);
                    solverConstraint.contactNormal.set(cp.normalWorldOnB);
                    solverConstraint.relpos1CrossNormal.cross(rel_pos1, cp.normalWorldOnB);
                    solverConstraint.relpos2CrossNormal.cross(rel_pos2, cp.normalWorldOnB);
                    if (rb0 != null) {
                        rb0.getVelocityInLocalPoint(rel_pos1, vel1);
                    } else {
                        vel1.set(0f, 0f, 0f);
                    }
                    if (rb1 != null) {
                        rb1.getVelocityInLocalPoint(rel_pos2, vel2);
                    } else {
                        vel2.set(0f, 0f, 0f);
                    }
                    vel.set(vel1).sub(vel2);
                    rel_vel = cp.normalWorldOnB.dot(vel);
                    solverConstraint.penetration = Math.min(cp.getDistance() + infoGlobal.linearSlop, 0f);
                    solverConstraint.friction = cp.combinedFriction;
                    solverConstraint.restitution = restitutionCurve(rel_vel, cp.combinedRestitution);
                    if (solverConstraint.restitution <= 0f) {
                        solverConstraint.restitution = 0f;
                    }
                    var penVel = -solverConstraint.penetration / infoGlobal.timeStep;
                    if (solverConstraint.restitution > penVel) {
                        solverConstraint.penetration = 0f;
                    }
                    solverConstraint.appliedImpulse = 0f;
                    solverConstraint.appliedPushImpulse = 0f;
                    solverConstraint.frictionIndex = tmpSolverFrictionConstraintPool.size();
                    if (!cp.lateralFrictionInitialized) {
                        cp.lateralFrictionDir1.set(cp.normalWorldOnB).mul(rel_vel);
                        cp.lateralFrictionDir1.set(vel).sub(cp.lateralFrictionDir1);
                        var lat_rel_vel = cp.lateralFrictionDir1.lengthSquared();
                        if (lat_rel_vel > BulletGlobals.EPSILON) {
                            cp.lateralFrictionDir1.mul(1f / (float) Math.sqrt(lat_rel_vel));
                            addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation);
                            cp.lateralFrictionDir2.cross(cp.lateralFrictionDir1, cp.normalWorldOnB);
                            cp.lateralFrictionDir2.normalize();
                            addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation);
                        } else {
                            Transform.planeSpace1(cp.normalWorldOnB, cp.lateralFrictionDir1, cp.lateralFrictionDir2);
                            addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation);
                            addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation);
                        }
                        cp.lateralFrictionInitialized = true;
                    } else {
                        addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation);
                        addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation);
                    }
                    var frictionConstraint1 = tmpSolverFrictionConstraintPool.get(solverConstraint.frictionIndex);
                    frictionConstraint1.appliedImpulse = 0f;
                    var frictionConstraint2 = tmpSolverFrictionConstraintPool.get(solverConstraint.frictionIndex + 1);
                    frictionConstraint2.appliedImpulse = 0f;
                }
            }
        }

        var numConstraintPool = tmpSolverConstraintPool.size();
        var numFrictionPool = tmpSolverFrictionConstraintPool.size();
        for (i = 0; i < numConstraintPool; i++) {
            orderTmpConstraintPool[i] = i;
        }
        for (i = 0; i < numFrictionPool; i++) {
            orderFrictionConstraintPool[i] = i;
        }
    }

    public float solveGroupCacheFriendlyIterations(ContactSolverInfo infoGlobal) {
        int iteration;
        for (iteration = 0; iteration < infoGlobal.numIterations; iteration++) {
            int j;
            var numPoolConstraints = tmpSolverConstraintPool.size();
            for (j = 0; j < numPoolConstraints; j++) {
                var solveManifold = tmpSolverConstraintPool.get(orderTmpConstraintPool[j]);
                resolveSingleCollisionCombinedCacheFriendly(tmpSolverBodyPool.get(solveManifold.solverBodyIdA), tmpSolverBodyPool.get(solveManifold.solverBodyIdB), solveManifold, infoGlobal);
            }
            var numFrictionPoolConstraints = tmpSolverFrictionConstraintPool.size();
            for (j = 0; j < numFrictionPoolConstraints; j++) {
                var solveManifold = tmpSolverFrictionConstraintPool.get(orderFrictionConstraintPool[j]);
                var totalImpulse = tmpSolverConstraintPool.get(solveManifold.frictionIndex).appliedImpulse + tmpSolverConstraintPool.get(solveManifold.frictionIndex).appliedPushImpulse;
                resolveSingleFrictionCacheFriendly(tmpSolverBodyPool.get(solveManifold.solverBodyIdA), tmpSolverBodyPool.get(solveManifold.solverBodyIdB), solveManifold, totalImpulse);
            }
        }
        if (infoGlobal.splitImpulse) {
            for (iteration = 0; iteration < infoGlobal.numIterations; iteration++) {
                var numPoolConstraints = tmpSolverConstraintPool.size();
                int j;
                for (j = 0; j < numPoolConstraints; j++) {
                    var solveManifold = tmpSolverConstraintPool.get(orderTmpConstraintPool[j]);
                    resolveSplitPenetrationImpulseCacheFriendly(tmpSolverBodyPool.get(solveManifold.solverBodyIdA), tmpSolverBodyPool.get(solveManifold.solverBodyIdB), solveManifold, infoGlobal);
                }
            }
        }
        return 0f;
    }

    public float solveGroupCacheFriendly(List<PersistentManifold> manifoldPtr, int manifold_offset, int numManifolds, int numConstraints, ContactSolverInfo infoGlobal) {
        solveGroupCacheFriendlySetup(manifoldPtr, manifold_offset, numManifolds, numConstraints, infoGlobal);
        solveGroupCacheFriendlyIterations(infoGlobal);
        for (SolverConstraint solveManifold : tmpSolverConstraintPool) {
            var pt = (ManifoldPoint) solveManifold.originalContactPoint;

            pt.appliedImpulse = solveManifold.appliedImpulse;
            pt.appliedImpulseLateral1 = tmpSolverFrictionConstraintPool.get(solveManifold.frictionIndex).appliedImpulse;
            pt.appliedImpulseLateral1 = tmpSolverFrictionConstraintPool.get(solveManifold.frictionIndex + 1).appliedImpulse;
        }
        if (infoGlobal.splitImpulse) {
            for (SolverBody solverBody : tmpSolverBodyPool) {
                solverBody.writebackVelocity(infoGlobal.timeStep);
            }
        } else {
            for (SolverBody solverBody : tmpSolverBodyPool) {
                solverBody.writebackVelocity();
            }
        }
        tmpSolverBodyPool.clear();
        tmpSolverConstraintPool.clear();
        tmpSolverFrictionConstraintPool.clear();
        return 0f;
    }

    @Override
    public float solveGroup(List<CollisionObject> bodies, int numBodies, List<PersistentManifold> manifoldPtr, int manifoldOffset, int numManifolds, ContactSolverInfo infoGlobal) {
        return solveGroupCacheFriendly(manifoldPtr, manifoldOffset, numManifolds, 0, infoGlobal /*,stackAlloc*/);
    }
}
