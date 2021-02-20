package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.PersistentManifold;

import java.util.List;

public interface ConstraintSolver {
    float solveGroup(List<CollisionObject> bodies, int numBodies, List<PersistentManifold> manifold, int manifoldOffset, int numManifolds, ContactSolverInfo info);
}
