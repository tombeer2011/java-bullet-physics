package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.Transform;
import com.bulletphysics.collision.shapes.ConvexShape;

import javax.vecmath.Vector3;

public class GjkEpaPenetrationDepthSolver implements ConvexPenetrationDepthSolver {
    private final GjkEpaSolver gjkEpaSolver = new GjkEpaSolver();

    public boolean calcPenDepth(ConvexShape pConvexA, ConvexShape pConvexB, Transform transformA, Transform transformB, Vector3 wWitnessOnA, Vector3 wWitnessOnB) {
        var radialmargin = 0f;
        var results = new GjkEpaSolver.Results();
        if (gjkEpaSolver.collide(pConvexA, transformA, pConvexB, transformB, radialmargin, results)) {
            wWitnessOnA.set(results.witnesses[0]);
            wWitnessOnB.set(results.witnesses[1]);
            return true;
        }
        return false;
    }
}
