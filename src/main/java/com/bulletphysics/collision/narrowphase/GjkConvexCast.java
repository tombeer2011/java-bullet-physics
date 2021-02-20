package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.Transform;
import com.bulletphysics.collision.narrowphase.DiscreteCollisionDetectorInterface.ClosestPointInput;
import com.bulletphysics.collision.shapes.ConvexShape;

import javax.vecmath.Vector3;

public class GjkConvexCast implements ConvexCast {
    private static final int MAX_ITERATIONS = 32;
    private final SimplexSolverInterface simplexSolver;
    private final ConvexShape convexA;
    private final ConvexShape convexB;
    private final GjkPairDetector gjk = new GjkPairDetector();

    public GjkConvexCast(ConvexShape convexA, ConvexShape convexB, SimplexSolverInterface simplexSolver) {
        this.simplexSolver = simplexSolver;
        this.convexA = convexA;
        this.convexB = convexB;
    }

    public boolean calcTimeOfImpact(Transform fromA, Transform toA, Transform fromB, Transform toB, CastResult result) {
        simplexSolver.reset();
        final var linVelA = new Vector3(toA.origin).sub(fromA.origin);
        final var linVelB = new Vector3(toB.origin).sub(fromB.origin);
        final var radius = 0.001f;
        final var n = new Vector3();
        final var c = new Vector3();
        final var r = new Vector3(linVelA).sub(linVelB);
        var lambda = 0f;
        var hasResult = false;
        var lastLambda = lambda;
        var numIter = 0;
        var input = new ClosestPointInput();
        input.transformA.set(fromA);
        input.transformB.set(fromB);
        var pointCollector = new PointCollector();
        gjk.init(convexA, convexB, simplexSolver, null);
        gjk.getClosestPoints(input, pointCollector);
        hasResult = pointCollector.hasResult;
        c.set(pointCollector.pointInWorld);
        if (hasResult) {
            n.set(pointCollector.normalOnBInWorld);
            float dist = pointCollector.distance;
            while (dist > radius) {
                if (++numIter > MAX_ITERATIONS) return false;
                var projectedLinearVelocity = r.dot(n);
                var dLambda = dist / projectedLinearVelocity;
                lambda -= dLambda;
                if (lambda > 1f) return false;
                if (lambda < 0f) return false;
                if (lambda <= lastLambda) return false;
                lastLambda = lambda;
                input.transformA.origin.interpolate(lambda, fromA.origin, toA.origin);
                input.transformB.origin.interpolate(lambda, fromB.origin, toB.origin);
                gjk.getClosestPoints(input, pointCollector);
                if (pointCollector.hasResult) {
                    if (pointCollector.distance < 0f) {
                        result.fraction = lastLambda;
                        n.set(pointCollector.normalOnBInWorld);
                        result.normal.set(n);
                        result.hitPoint.set(pointCollector.pointInWorld);
                        return true;
                    }
                    c.set(pointCollector.pointInWorld);
                    n.set(pointCollector.normalOnBInWorld);
                    dist = pointCollector.distance;
                } else return false;
            }
            if (n.dot(r) >= -result.allowedPenetration) return false;
            result.fraction = lambda;
            result.normal.set(n);
            result.hitPoint.set(c);
            return true;
        }
        return false;
    }
}
