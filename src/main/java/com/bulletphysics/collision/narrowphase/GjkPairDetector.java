package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.Transform;
import com.bulletphysics.collision.shapes.ConvexShape;

import javax.vecmath.Vector3;

public class GjkPairDetector extends DiscreteCollisionDetectorInterface {
    private static final float REL_ERROR2 = 1.0e-6f;
    private final Vector3 cachedSeparatingAxis = new Vector3();
    private ConvexPenetrationDepthSolver penetrationDepthSolver;
    private SimplexSolverInterface simplexSolver;
    private ConvexShape minkowskiA;
    private ConvexShape minkowskiB;
    private boolean ignoreMargin;
    public int lastUsedMethod;
    public int curIter;
    public int degenerateSimplex;
    public int catchDegeneracies;

    public void init(ConvexShape objectA, ConvexShape objectB, SimplexSolverInterface simplexSolver, ConvexPenetrationDepthSolver penetrationDepthSolver) {
        cachedSeparatingAxis.set(0f, 0f, 1f);
        ignoreMargin = false;
        lastUsedMethod = -1;
        catchDegeneracies = 1;
        this.penetrationDepthSolver = penetrationDepthSolver;
        this.simplexSolver = simplexSolver;
        minkowskiA = objectA;
        minkowskiB = objectB;
    }

    public void getClosestPoints(ClosestPointInput input, Result output) {
        var tmp = new Vector3();
        var distance = 0f;
        var normalInB = new Vector3();
        var pointOnA = new Vector3();
        var pointOnB = new Vector3();
        var localTransA = new Transform(input.transformA);
        var localTransB = new Transform(input.transformB);
        var positionOffset = new Vector3(localTransA.origin).add(localTransB.origin).mul(0.5f);
        localTransA.origin.sub(positionOffset);
        localTransB.origin.sub(positionOffset);
        var marginA = minkowskiA.getMargin();
        var marginB = minkowskiB.getMargin();
        if (ignoreMargin) {
            marginA = 0f;
            marginB = 0f;
        }
        curIter = 0;
        var gGjkMaxIter = 1000;
        cachedSeparatingAxis.set(0f, 1f, 0f);
        var isValid = false;
        var checkSimplex = false;
        var checkPenetration = true;
        degenerateSimplex = 0;
        lastUsedMethod = -1;
        var squaredDistance = BulletGlobals.INFINITY;
        var delta = 0f;
        var margin = marginA + marginB;
        simplexSolver.reset();
        var seperatingAxisInA = new Vector3();
        var seperatingAxisInB = new Vector3();
        var pInA = new Vector3();
        var qInB = new Vector3();
        var pWorld = new Vector3();
        var qWorld = new Vector3();
        var w = new Vector3();
        var tmpPointOnA = new Vector3();
        var tmpPointOnB = new Vector3();
        var tmpNormalInB = new Vector3();
        for (; ; ) {
            seperatingAxisInA.set(cachedSeparatingAxis).negate();
            input.transformA.basis.transposeTransform(seperatingAxisInA, seperatingAxisInA);
            seperatingAxisInB.set(cachedSeparatingAxis);
            input.transformB.basis.transposeTransform(seperatingAxisInB, seperatingAxisInB);
            minkowskiA.localGetSupportingVertexWithoutMargin(seperatingAxisInA, pInA);
            minkowskiB.localGetSupportingVertexWithoutMargin(seperatingAxisInB, qInB);
            localTransA.transform(pWorld.set(pInA));
            localTransB.transform(qWorld.set(qInB));
            delta = cachedSeparatingAxis.dot(w.set(pWorld).sub(qWorld));
            if (delta > 0f && delta * delta > squaredDistance * input.maximumDistanceSquared) {
                checkPenetration = false;
                break;
            }
            if (simplexSolver.inSimplex(w)) {
                degenerateSimplex = 1;
                checkSimplex = true;
                break;
            }
            var f0 = squaredDistance - delta;
            var f1 = squaredDistance * REL_ERROR2;
            if (f0 <= f1) {
                if (f0 <= 0f) {
                    degenerateSimplex = 2;
                }
                checkSimplex = true;
                break;
            }
            simplexSolver.addVertex(w, pWorld, qWorld);
            if (!simplexSolver.closest(cachedSeparatingAxis)) {
                degenerateSimplex = 3;
                checkSimplex = true;
                break;
            }
            if (cachedSeparatingAxis.lengthSquared() < REL_ERROR2) {
                degenerateSimplex = 6;
                checkSimplex = true;
                break;
            }
            var previousSquaredDistance = squaredDistance;
            squaredDistance = cachedSeparatingAxis.lengthSquared();
            if (previousSquaredDistance - squaredDistance <= BulletGlobals.EPSILON * previousSquaredDistance) {
                simplexSolver.backup_closest(cachedSeparatingAxis);
                checkSimplex = true;
                break;
            }
            if (curIter++ > gGjkMaxIter) break;
            var check = !simplexSolver.fullSimplex();
            if (!check) {
                simplexSolver.backup_closest(cachedSeparatingAxis);
                break;
            }
        }
        if (checkSimplex) {
            simplexSolver.compute_points(pointOnA, pointOnB);
            normalInB.set(pointOnA).sub(pointOnB);
            var lenSqr = cachedSeparatingAxis.lengthSquared();
            if (lenSqr < 0.0001f) degenerateSimplex = 5;
            if (lenSqr > BulletGlobals.EPSILON * BulletGlobals.EPSILON) {
                var rlen = 1f / (float) Math.sqrt(lenSqr);
                normalInB.mul(rlen);
                var s = (float) Math.sqrt(squaredDistance);
                pointOnA.sub(tmp.set(cachedSeparatingAxis).mul(marginA / s));
                pointOnB.add(tmp.set(cachedSeparatingAxis).mul(marginB / s));
                distance = 1f / rlen - margin;
                isValid = true;
                lastUsedMethod = 1;
            } else {
                lastUsedMethod = 2;
            }
        }
        var catchDegeneratePenetrationCase = catchDegeneracies != 0 && penetrationDepthSolver != null && degenerateSimplex != 0 && distance + margin < 0.01f;
        if (checkPenetration && (!isValid || catchDegeneratePenetrationCase)) {
            if (penetrationDepthSolver != null) {
                var isValid2 = penetrationDepthSolver.calcPenDepth(minkowskiA, minkowskiB, localTransA, localTransB, tmpPointOnA, tmpPointOnB);
                if (isValid2) {
                    tmpNormalInB.set(tmpPointOnB).sub(tmpPointOnA);
                    var lenSqr = tmpNormalInB.lengthSquared();
                    if (lenSqr > BulletGlobals.EPSILON * BulletGlobals.EPSILON) {
                        tmpNormalInB.mul(1f / (float) Math.sqrt(lenSqr));
                        tmp.set(tmpPointOnA).sub(tmpPointOnB);
                        var distance2 = -tmp.length();
                        if (!isValid || distance2 < distance) {
                            distance = distance2;
                            pointOnA.set(tmpPointOnA);
                            pointOnB.set(tmpPointOnB);
                            normalInB.set(tmpNormalInB);
                            isValid = true;
                            lastUsedMethod = 3;
                        }
                    } else lastUsedMethod = 4;
                } else lastUsedMethod = 5;
            }
        }
        if (isValid) {
            output.addContactPoint(normalInB, tmp.set(pointOnB).add(positionOffset), distance);
        }
    }

    public void setMinkowskiA(ConvexShape minkA) {
        minkowskiA = minkA;
    }

    public void setMinkowskiB(ConvexShape minkB) {
        minkowskiB = minkB;
    }
}
