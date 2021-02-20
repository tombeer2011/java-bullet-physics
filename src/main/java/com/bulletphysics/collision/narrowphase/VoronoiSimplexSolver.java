package com.bulletphysics.collision.narrowphase;

import javax.vecmath.Vector3;

public class VoronoiSimplexSolver implements SimplexSolverInterface {
    private static final int VORONOI_SIMPLEX_MAX_VERTS = 5;
    private static final int VERTA = 0;
    private static final int VERTB = 1;
    private static final int VERTC = 2;
    public int numVertices;
    public final Vector3[] simplexVectorW = new Vector3[VORONOI_SIMPLEX_MAX_VERTS];
    public final Vector3[] simplexPointsP = new Vector3[VORONOI_SIMPLEX_MAX_VERTS];
    public final Vector3[] simplexPointsQ = new Vector3[VORONOI_SIMPLEX_MAX_VERTS];
    public final Vector3 cachedP1 = new Vector3();
    public final Vector3 cachedP2 = new Vector3();
    public final Vector3 cachedV = new Vector3();
    public final Vector3 lastW = new Vector3();
    public boolean cachedValidClosest;
    public final SubSimplexClosestResult cachedBC = new SubSimplexClosestResult();
    public boolean needsUpdate;

    {
        for (var i = 0; i < VORONOI_SIMPLEX_MAX_VERTS; i++) {
            simplexVectorW[i] = new Vector3();
            simplexPointsP[i] = new Vector3();
            simplexPointsQ[i] = new Vector3();
        }
    }

    public void removeVertex(int index) {

        numVertices--;
        simplexVectorW[index].set(simplexVectorW[numVertices]);
        simplexPointsP[index].set(simplexPointsP[numVertices]);
        simplexPointsQ[index].set(simplexPointsQ[numVertices]);
    }

    public void reduceVertices(UsageBitfield usedVerts) {
        if (numVertices() >= 4 && !usedVerts.usedVertexD) {
            removeVertex(3);
        }
        if (numVertices() >= 3 && !usedVerts.usedVertexC) {
            removeVertex(2);
        }
        if (numVertices() >= 2 && !usedVerts.usedVertexB) {
            removeVertex(1);
        }
        if (numVertices() >= 1 && !usedVerts.usedVertexA) {
            removeVertex(0);
        }
    }

    public boolean updateClosestVectorAndPoints() {
        if (needsUpdate) {
            cachedBC.reset();
            needsUpdate = false;
            switch (numVertices()) {
                case 1:
                    cachedP1.set(simplexPointsP[0]);
                    cachedP2.set(simplexPointsQ[0]);
                    cachedV.set(cachedP1).sub(cachedP2);
                    cachedBC.reset();
                    cachedBC.setBarycentricCoordinates(1f, 0f, 0f, 0f);
                    cachedValidClosest = cachedBC.isValid();
                    break;
                case 2: {
                    var tmp = new Vector3();
                    var from = simplexVectorW[0];
                    var to = simplexVectorW[1];
                    var nearest = new Vector3();
                    var p = new Vector3();
                    p.set(0f, 0f, 0f);
                    var diff = new Vector3();
                    diff.set(p).sub(from);
                    var v = new Vector3();
                    v.set(to).sub(from);
                    var t = v.dot(diff);
                    if (t > 0) {
                        var dotVV = v.dot(v);
                        if (t < dotVV) {
                            t /= dotVV;
                            tmp.set(v).mul(t);
                            diff.sub(tmp);
                            cachedBC.usedVertices.usedVertexA = true;
                            cachedBC.usedVertices.usedVertexB = true;
                        } else {
                            t = 1;
                            diff.sub(v);
                            cachedBC.usedVertices.usedVertexB = true;
                        }
                    } else {
                        t = 0;
                        cachedBC.usedVertices.usedVertexA = true;
                    }
                    cachedBC.setBarycentricCoordinates(1f - t, t, 0f, 0f);
                    tmp.set(v).mul(t);
                    nearest.set(from).add(tmp);
                    tmp.set(simplexPointsP[1]).sub(simplexPointsP[0]);
                    tmp.mul(t);
                    cachedP1.set(simplexPointsP[0]).add(tmp);
                    tmp.set(simplexPointsQ[1]).sub(simplexPointsQ[0]);
                    tmp.mul(t);
                    cachedP2.set(simplexPointsQ[0]).add(tmp);
                    cachedV.set(cachedP1).sub(cachedP2);
                    reduceVertices(cachedBC.usedVertices);
                    cachedValidClosest = cachedBC.isValid();
                    break;
                }
                case 3: {
                    var tmp1 = new Vector3();
                    var tmp2 = new Vector3();
                    var tmp3 = new Vector3();
                    var p = new Vector3();
                    p.set(0f, 0f, 0f);
                    var a = simplexVectorW[0];
                    var b = simplexVectorW[1];
                    var c = simplexVectorW[2];
                    closestPtPointTriangle(p, a, b, c, cachedBC);
                    tmp1.set(simplexPointsP[0]).mul(cachedBC.barycentricCoords[0]);
                    tmp2.set(simplexPointsP[1]).mul(cachedBC.barycentricCoords[1]);
                    tmp3.set(simplexPointsP[2]).mul(cachedBC.barycentricCoords[2]);
                    cachedP1.set(tmp1).add(tmp2).add(tmp3);
                    tmp1.set(simplexPointsQ[0]).mul(cachedBC.barycentricCoords[0]);
                    tmp2.set(simplexPointsQ[1]).mul(cachedBC.barycentricCoords[1]);
                    tmp3.set(simplexPointsQ[2]).mul(cachedBC.barycentricCoords[2]);
                    cachedP2.set(tmp1).add(tmp2).add(tmp3);
                    cachedV.set(cachedP1).sub(cachedP2);
                    reduceVertices(cachedBC.usedVertices);
                    cachedValidClosest = cachedBC.isValid();
                    break;
                }
                case 4:
                    var tmp1 = new Vector3();
                    var tmp2 = new Vector3();
                    var tmp3 = new Vector3();
                    var tmp4 = new Vector3();
                    var p = new Vector3();
                    p.set(0f, 0f, 0f);
                    var a = simplexVectorW[0];
                    var b = simplexVectorW[1];
                    var c = simplexVectorW[2];
                    var d = simplexVectorW[3];
                    var hasSeperation = closestPtPointTetrahedron(p, a, b, c, d, cachedBC);
                    if (hasSeperation) {
                        tmp1.set(simplexPointsP[0]).mul(cachedBC.barycentricCoords[0]);
                        tmp2.set(simplexPointsP[1]).mul(cachedBC.barycentricCoords[1]);
                        tmp3.set(simplexPointsP[2]).mul(cachedBC.barycentricCoords[2]);
                        tmp4.set(simplexPointsP[3]).mul(cachedBC.barycentricCoords[3]);
                        cachedP1.set(tmp1).add(tmp2).add(tmp3).add(tmp4);
                        tmp1.set(simplexPointsQ[0]).mul(cachedBC.barycentricCoords[0]);
                        tmp2.set(simplexPointsQ[1]).mul(cachedBC.barycentricCoords[1]);
                        tmp3.set(simplexPointsQ[2]).mul(cachedBC.barycentricCoords[2]);
                        tmp4.set(simplexPointsQ[3]).mul(cachedBC.barycentricCoords[3]);
                        cachedP2.set(tmp1).add(tmp2).add(tmp3).add(tmp4);
                        cachedV.set(cachedP1).sub(cachedP2);
                        reduceVertices(cachedBC.usedVertices);
                    } else {
                        if (cachedBC.degenerate) {
                            cachedValidClosest = false;
                        } else {
                            cachedValidClosest = true;
                            cachedV.set(0f, 0f, 0f);
                        }
                        break;
                    }
                    cachedValidClosest = cachedBC.isValid();
                    break;
                case 0:
                default:
                    cachedValidClosest = false;
            }
        }
        return cachedValidClosest;
    }

    public boolean closestPtPointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c, SubSimplexClosestResult result) {
        result.usedVertices.reset();
        var ab = new Vector3();
        ab.set(b).sub(a);
        var ac = new Vector3();
        ac.set(c).sub(a);
        var ap = new Vector3();
        ap.set(p).sub(a);
        var d1 = ab.dot(ap);
        var d2 = ac.dot(ap);
        if (d1 <= 0f && d2 <= 0f) {
            result.closestPointOnSimplex.set(a);
            result.usedVertices.usedVertexA = true;
            result.setBarycentricCoordinates(1f, 0f, 0f, 0f);
            return true;
        }
        var bp = new Vector3();
        bp.set(p).sub(b);
        var d3 = ab.dot(bp);
        var d4 = ac.dot(bp);
        if (d3 >= 0f && d4 <= d3) {
            result.closestPointOnSimplex.set(b);
            result.usedVertices.usedVertexB = true;
            result.setBarycentricCoordinates(0, 1f, 0f, 0f);
            return true;
        }
        var vc = d1 * d4 - d3 * d2;
        if (vc <= 0f && d1 >= 0f && d3 <= 0f) {
            var v = d1 / (d1 - d3);
            result.closestPointOnSimplex.scaleAdd(v, ab, a);
            result.usedVertices.usedVertexA = true;
            result.usedVertices.usedVertexB = true;
            result.setBarycentricCoordinates(1f - v, v, 0f, 0f);
            return true;
        }
        var cp = new Vector3();
        cp.set(p).sub(c);
        var d5 = ab.dot(cp);
        var d6 = ac.dot(cp);
        if (d6 >= 0f && d5 <= d6) {
            result.closestPointOnSimplex.set(c);
            result.usedVertices.usedVertexC = true;
            result.setBarycentricCoordinates(0f, 0f, 1f, 0f);
            return true;
        }
        var vb = d5 * d2 - d1 * d6;
        if (vb <= 0f && d2 >= 0f && d6 <= 0f) {
            var w = d2 / (d2 - d6);
            result.closestPointOnSimplex.scaleAdd(w, ac, a);
            result.usedVertices.usedVertexA = true;
            result.usedVertices.usedVertexC = true;
            result.setBarycentricCoordinates(1f - w, 0f, w, 0f);
            return true;
        }
        var va = d3 * d6 - d5 * d4;
        if (va <= 0f && d4 - d3 >= 0f && d5 - d6 >= 0f) {
            var w = (d4 - d3) / (d4 - d3 + (d5 - d6));
            var tmp = new Vector3();
            tmp.set(c).sub(b);
            result.closestPointOnSimplex.scaleAdd(w, tmp, b);
            result.usedVertices.usedVertexB = true;
            result.usedVertices.usedVertexC = true;
            result.setBarycentricCoordinates(0, 1f - w, w, 0f);
            return true;
        }
        var denom = 1f / (va + vb + vc);
        var v = vb * denom;
        var w = vc * denom;
        var tmp1 = new Vector3();
        var tmp2 = new Vector3();
        tmp1.set(ab).mul(v);
        tmp2.set(ac).mul(w);
        result.closestPointOnSimplex.set(a).add(tmp1).add(tmp2);
        result.usedVertices.usedVertexA = true;
        result.usedVertices.usedVertexB = true;
        result.usedVertices.usedVertexC = true;
        result.setBarycentricCoordinates(1f - v - w, v, w, 0f);
        return true;
    }

    public static int pointOutsideOfPlane(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
        var tmp = new Vector3();
        var normal = new Vector3();
        normal.set(b).sub(a);
        tmp.set(c).sub(a);
        normal.cross(normal, tmp);
        tmp.set(p).sub(a);
        var signp = tmp.dot(normal);
        tmp.set(d).sub(a);
        var signd = tmp.dot(normal);
        if (signd * signd < 1e-4f * 1e-4f) {
            return -1;
        }
        return signp * signd < 0f ? 1 : 0;
    }

    public boolean closestPtPointTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d, SubSimplexClosestResult finalResult) {
        var tempResult = new SubSimplexClosestResult();
        tempResult.reset();
        var tmp = new Vector3();
        var q = new Vector3();
        finalResult.closestPointOnSimplex.set(p);
        finalResult.usedVertices.reset();
        finalResult.usedVertices.usedVertexA = true;
        finalResult.usedVertices.usedVertexB = true;
        finalResult.usedVertices.usedVertexC = true;
        finalResult.usedVertices.usedVertexD = true;
        var pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
        var pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
        var pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
        var pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);
        if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0) {
            finalResult.degenerate = true;
            return false;
        }
        if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0) {
            return false;
        }
        var bestSqDist = Float.MAX_VALUE;
        if (pointOutsideABC != 0) {
            closestPtPointTriangle(p, a, b, c, tempResult);
            q.set(tempResult.closestPointOnSimplex);
            tmp.set(q).sub(p);
            var sqDist = tmp.dot(tmp);
            if (sqDist < bestSqDist) {
                bestSqDist = sqDist;
                finalResult.closestPointOnSimplex.set(q);
                finalResult.usedVertices.reset();
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexB;
                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
                finalResult.setBarycentricCoordinates(tempResult.barycentricCoords[VERTA], tempResult.barycentricCoords[VERTB], tempResult.barycentricCoords[VERTC], 0);
            }
        }
        if (pointOutsideACD != 0) {
            closestPtPointTriangle(p, a, c, d, tempResult);
            q.set(tempResult.closestPointOnSimplex);
            tmp.set(q).sub(p);
            var sqDist = tmp.dot(tmp);
            if (sqDist < bestSqDist) {
                bestSqDist = sqDist;
                finalResult.closestPointOnSimplex.set(q);
                finalResult.usedVertices.reset();
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexB;
                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexC;
                finalResult.setBarycentricCoordinates(tempResult.barycentricCoords[VERTA], 0, tempResult.barycentricCoords[VERTB], tempResult.barycentricCoords[VERTC]);
            }
        }
        if (pointOutsideADB != 0) {
            closestPtPointTriangle(p, a, d, b, tempResult);
            q.set(tempResult.closestPointOnSimplex);
            tmp.set(q).sub(p);
            var sqDist = tmp.dot(tmp);
            if (sqDist < bestSqDist) {
                bestSqDist = sqDist;
                finalResult.closestPointOnSimplex.set(q);
                finalResult.usedVertices.reset();
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexC;
                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;
                finalResult.setBarycentricCoordinates(tempResult.barycentricCoords[VERTA], tempResult.barycentricCoords[VERTC], 0, tempResult.barycentricCoords[VERTB]);
            }
        }
        if (pointOutsideBDC != 0) {
            closestPtPointTriangle(p, b, d, c, tempResult);
            q.set(tempResult.closestPointOnSimplex);
            tmp.set(q).sub(p);
            var sqDist = tmp.dot(tmp);
            if (sqDist < bestSqDist) {
                finalResult.closestPointOnSimplex.set(q);
                finalResult.usedVertices.reset();
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexA;
                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;
                finalResult.setBarycentricCoordinates(0, tempResult.barycentricCoords[VERTA], tempResult.barycentricCoords[VERTC], tempResult.barycentricCoords[VERTB]);
            }
        }
        if (finalResult.usedVertices.usedVertexA && finalResult.usedVertices.usedVertexB && finalResult.usedVertices.usedVertexC && finalResult.usedVertices.usedVertexD) {
            return true;
        }
        return true;
    }

    public void reset() {
        cachedValidClosest = false;
        numVertices = 0;
        needsUpdate = true;
        lastW.set(1e30f, 1e30f, 1e30f);
        cachedBC.reset();
    }

    public void addVertex(Vector3 w, Vector3 p, Vector3 q) {
        lastW.set(w);
        needsUpdate = true;
        simplexVectorW[numVertices].set(w);
        simplexPointsP[numVertices].set(p);
        simplexPointsQ[numVertices].set(q);
        numVertices++;
    }

    public boolean closest(Vector3 v) {
        var succes = updateClosestVectorAndPoints();
        v.set(cachedV);
        return succes;
    }

    public boolean fullSimplex() {
        return numVertices == 4;
    }

    public boolean inSimplex(Vector3 w) {
        var found = false;
        int i, numverts = numVertices();
        for (i = 0; i < numverts; i++) {
            if (simplexVectorW[i].equals(w)) {
                found = true;
            }
        }
        if (w.equals(lastW)) {
            return true;
        }
        return found;
    }

    public void backup_closest(Vector3 v) {
        v.set(cachedV);
    }

    public void compute_points(Vector3 p1, Vector3 p2) {
        updateClosestVectorAndPoints();
        p1.set(cachedP1);
        p2.set(cachedP2);
    }

    public int numVertices() {
        return numVertices;
    }

    public static class UsageBitfield {
        public boolean usedVertexA;
        public boolean usedVertexB;
        public boolean usedVertexC;
        public boolean usedVertexD;

        public void reset() {
            usedVertexA = false;
            usedVertexB = false;
            usedVertexC = false;
            usedVertexD = false;
        }
    }

    public static class SubSimplexClosestResult {
        public final Vector3 closestPointOnSimplex = new Vector3();
        public final UsageBitfield usedVertices = new UsageBitfield();
        public final float[] barycentricCoords = new float[4];
        public boolean degenerate;

        public void reset() {
            degenerate = false;
            setBarycentricCoordinates(0f, 0f, 0f, 0f);
            usedVertices.reset();
        }

        public boolean isValid() {
            return barycentricCoords[0] >= 0f && barycentricCoords[1] >= 0f && barycentricCoords[2] >= 0f && barycentricCoords[3] >= 0f;
        }

        public void setBarycentricCoordinates(float a, float b, float c, float d) {
            barycentricCoords[0] = a;
            barycentricCoords[1] = b;
            barycentricCoords[2] = c;
            barycentricCoords[3] = d;
        }
    }
}
