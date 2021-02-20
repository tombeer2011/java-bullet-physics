package com.bulletphysics.dynamics.constraintsolver;

import javax.vecmath.Vector3;

public class SolverConstraint {
    public final Vector3 relpos1CrossNormal = new Vector3();
    public final Vector3 contactNormal = new Vector3();
    public final Vector3 relpos2CrossNormal = new Vector3();
    public final Vector3 angularComponentA = new Vector3();
    public final Vector3 angularComponentB = new Vector3();
    public float appliedPushImpulse;
    public float appliedImpulse;
    public int solverBodyIdA;
    public int solverBodyIdB;
    public float friction;
    public float restitution;
    public float jacDiagABInv;
    public float penetration;
    public SolverConstraintType constraintType;
    public int frictionIndex;
    public Object originalContactPoint;
}
