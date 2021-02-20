package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.Transform;
import com.bulletphysics.collision.shapes.ConvexShape;

import javax.vecmath.Vector3;

public interface ConvexPenetrationDepthSolver {
    boolean calcPenDepth(ConvexShape convexA, ConvexShape convexB, Transform transA, Transform transB, Vector3 pa, Vector3 pb);
}
