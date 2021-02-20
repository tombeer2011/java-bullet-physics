package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.Transform;
import com.bulletphysics.collision.shapes.ConvexShape;

import javax.vecmath.Matrix3;
import javax.vecmath.Quaternion;
import javax.vecmath.Vector3;
import java.util.Arrays;
import java.util.Objects;

public class GjkEpaSolver {

    public enum ResultsStatus {
        Separated, Penetrating, GJK_Failed, EPA_Failed,
    }

    public static class Results {
        public ResultsStatus status;
        public final Vector3[] witnesses = new Vector3[]{new Vector3(), new Vector3()};
        public final Vector3 normal = new Vector3();
        public float depth;
        public int epa_iterations;
        public int gjk_iterations;
    }

    private static final float cstInf = BulletGlobals.INFINITY;
    private static final float cst2Pi = BulletGlobals.TAU;
    private static final int GJK_maxiterations = 128;
    private static final int GJK_hashsize = 1 << 6;
    private static final int GJK_hashmask = GJK_hashsize - 1;
    private static final float GJK_insimplex_eps = 0.0001f;
    private static final float GJK_sqinsimplex_eps = GJK_insimplex_eps * GJK_insimplex_eps;
    private static final int EPA_maxiterations = 256;
    private static final float EPA_inface_eps = 0.01f;
    private static final float EPA_accuracy = 0.001f;

    public static class Mkv {
        public final Vector3 w = new Vector3();
        public final Vector3 r = new Vector3();

        public void set(Mkv m) {
            w.set(m.w);
            r.set(m.r);
        }
    }

    public static class He {
        public final Vector3 v = new Vector3();
        public He n;
    }

    protected static class GJK {
        public final He[] table = new He[GJK_hashsize];
        public final Matrix3[] wrotations = new Matrix3[]{new Matrix3(), new Matrix3()};
        public final Vector3[] positions = new Vector3[]{new Vector3(), new Vector3()};
        public final ConvexShape[] shapes = new ConvexShape[2];
        public final Mkv[] simplex = new Mkv[5];
        public final Vector3 ray = new Vector3();
        public int order;
        public int iterations;
        public float margin;
        public boolean failed;

        {
            for (var i = 0; i < simplex.length; i++) {
                simplex[i] = new Mkv();
            }
        }

        public GJK() {
        }

        public void init(Matrix3 wrot0, Vector3 pos0, ConvexShape shape0, Matrix3 wrot1, Vector3 pos1, ConvexShape shape1, float pmargin) {
            wrotations[0].set(wrot0);
            positions[0].set(pos0);
            shapes[0] = shape0;
            wrotations[1].set(wrot1);
            positions[1].set(pos1);
            shapes[1] = shape1;
            margin = pmargin;
            failed = false;
        }

        public void destroy() {
        }

        public int Hash(Vector3 v) {
            var h = (int) (v.x * 15461) ^ (int) (v.y * 83003) ^ (int) (v.z * 15473);
            return h * 169639 & GJK_hashmask;
        }

        public Vector3 LocalSupport(Vector3 d, int i, Vector3 out) {
            var tmp = new Vector3();
            wrotations[i].transposeTransform(tmp, d);
            shapes[i].localGetSupportingVertex(tmp, out);
            out.transform(wrotations[i]);
            out.add(positions[i]);
            return out;
        }

        public void Support(Vector3 d, Mkv v) {
            v.r.set(d);
            var tmp1 = LocalSupport(d, 0, new Vector3());
            var tmp = new Vector3();
            tmp.set(d);
            tmp.negate();
            var tmp2 = LocalSupport(tmp, 1, new Vector3());
            v.w.set(tmp1).sub(tmp2);
            v.w.scaleAdd(margin, d, v.w);
        }

        public boolean FetchSupport() {
            var h = Hash(ray);
            var e = table[h];
            while (e != null) {
                if (e.v.equals(ray)) {
                    --order;
                    return false;
                } else {
                    e = e.n;
                }
            }
            e = new He();
            e.v.set(ray);
            e.n = table[h];
            table[h] = e;
            Support(ray, simplex[++order]);
            return ray.dot(simplex[order].w) > 0;
        }

        public boolean SolveSimplex2(Vector3 ao, Vector3 ab) {
            if (ab.dot(ao) >= 0) {
                var cabo = new Vector3();
                cabo.cross(ab, ao);
                if (cabo.lengthSquared() > GJK_sqinsimplex_eps) {
                    ray.cross(cabo, ab);
                } else {
                    return true;
                }
            } else {
                order = 0;
                simplex[0].set(simplex[1]);
                ray.set(ao);
            }
            return false;
        }

        public boolean SolveSimplex3(Vector3 ao, Vector3 ab, Vector3 ac) {
            var tmp = new Vector3();
            tmp.cross(ab, ac);
            return SolveSimplex3a(ao, ab, ac, tmp);
        }

        public boolean SolveSimplex3a(Vector3 ao, Vector3 ab, Vector3 ac, Vector3 cabc) {
            var tmp = new Vector3();
            tmp.cross(cabc, ab);
            var tmp2 = new Vector3();
            tmp2.cross(cabc, ac);
            if (tmp.dot(ao) < -GJK_insimplex_eps) {
                order = 1;
                simplex[0].set(simplex[1]);
                simplex[1].set(simplex[2]);
                return SolveSimplex2(ao, ab);
            } else if (tmp2.dot(ao) > +GJK_insimplex_eps) {
                order = 1;
                simplex[1].set(simplex[2]);
                return SolveSimplex2(ao, ac);
            } else {
                var d = cabc.dot(ao);
                if (Math.abs(d) > GJK_insimplex_eps) {
                    if (d > 0) {
                        ray.set(cabc);
                    } else {
                        ray.set(cabc).negate();
                        var swapTmp = new Mkv();
                        swapTmp.set(simplex[0]);
                        simplex[0].set(simplex[1]);
                        simplex[1].set(swapTmp);
                    }
                    return false;
                } else {
                    return true;
                }
            }
        }

        public boolean SolveSimplex4(Vector3 ao, Vector3 ab, Vector3 ac, Vector3 ad) {
            var crs = new Vector3();
            var tmp = new Vector3();
            tmp.cross(ab, ac);
            var tmp2 = new Vector3();
            tmp2.cross(ac, ad);
            var tmp3 = new Vector3();
            tmp3.cross(ad, ab);
            if (tmp.dot(ao) > GJK_insimplex_eps) {
                crs.set(tmp);
                order = 2;
                simplex[0].set(simplex[1]);
                simplex[1].set(simplex[2]);
                simplex[2].set(simplex[3]);
                return SolveSimplex3a(ao, ab, ac, crs);
            } else if (tmp2.dot(ao) > GJK_insimplex_eps) {
                crs.set(tmp2);
                order = 2;
                simplex[2].set(simplex[3]);
                return SolveSimplex3a(ao, ac, ad, crs);
            } else if (tmp3.dot(ao) > GJK_insimplex_eps) {
                crs.set(tmp3);
                order = 2;
                simplex[1].set(simplex[0]);
                simplex[0].set(simplex[2]);
                simplex[2].set(simplex[3]);
                return SolveSimplex3a(ao, ad, ab, crs);
            } else {
                return true;
            }
        }

        public boolean SearchOrigin() {
            var tmp = new Vector3();
            tmp.set(1f, 0f, 0f);
            return SearchOrigin(tmp);
        }

        public boolean SearchOrigin(Vector3 initray) {
            var tmp1 = new Vector3();
            var tmp2 = new Vector3();
            var tmp3 = new Vector3();
            var tmp4 = new Vector3();
            iterations = 0;
            order = -1;
            failed = false;
            ray.set(initray);
            ray.normalize();
            Arrays.fill(table, null);
            FetchSupport();
            ray.set(simplex[0].w).negate();
            for (; iterations < GJK_maxiterations; ++iterations) {
                var rl = ray.length();
                ray.mul(1f / (rl > 0f ? rl : 1f));
                if (FetchSupport()) {
                    var found = false;
                    switch (order) {
                        case 1:
                            tmp1.set(simplex[1].w).negate();
                            tmp2.set(simplex[0].w).sub(simplex[1].w);
                            found = SolveSimplex2(tmp1, tmp2);
                            break;
                        case 2:
                            tmp1.set(simplex[2].w).negate();
                            tmp2.set(simplex[1].w).sub(simplex[2].w);
                            tmp3.set(simplex[0].w).sub(simplex[2].w);
                            found = SolveSimplex3(tmp1, tmp2, tmp3);
                            break;
                        case 3:
                            tmp1.set(simplex[3].w).negate();
                            tmp2.set(simplex[2].w).sub(simplex[3].w);
                            tmp3.set(simplex[1].w).sub(simplex[3].w);
                            tmp4.set(simplex[0].w).sub(simplex[3].w);
                            found = SolveSimplex4(tmp1, tmp2, tmp3, tmp4);
                            break;
                    }
                    if (found) {
                        return true;
                    }
                } else {
                    return false;
                }
            }
            failed = true;
            return false;
        }

        public boolean EncloseOrigin() {
            var tmp = new Vector3();
            var tmp1 = new Vector3();
            var tmp2 = new Vector3();
            switch (order) {
                case 0:
                    break;
                case 1:
                    var ab = new Vector3();
                    ab.set(simplex[1].w).sub(simplex[0].w);
                    var b = new Vector3[]{new Vector3(), new Vector3(), new Vector3()};
                    b[0].set(1f, 0f, 0f);
                    b[1].set(0f, 1f, 0f);
                    b[2].set(0f, 0f, 1f);
                    b[0].cross(ab, b[0]);
                    b[1].cross(ab, b[1]);
                    b[2].cross(ab, b[2]);
                    var m = new float[]{b[0].lengthSquared(), b[1].lengthSquared(), b[2].lengthSquared()};
                    var tmpQuat = new Quaternion();
                    tmp.set(ab).normalize();
                    tmpQuat.setRotation(tmp, cst2Pi / 3f);
                    var r = new Matrix3();
                    r.setRotation(tmpQuat);
                    var w = new Vector3();
                    w.set(b[m[0] > m[1] ? m[0] > m[2] ? 0 : 2 : m[1] > m[2] ? 1 : 2]);
                    tmp.set(w).normalize();
                    Support(tmp, simplex[4]);
                    w.transform(r);
                    tmp.set(w).normalize();
                    Support(tmp, simplex[2]);
                    w.transform(r);
                    tmp.set(w).normalize();
                    Support(tmp, simplex[3]);
                    w.transform(r);
                    order = 4;
                    return true;
                case 2:
                    tmp1.set(simplex[1].w).sub(simplex[0].w);
                    tmp2.set(simplex[2].w).sub(simplex[0].w);
                    var n = new Vector3();
                    n.cross(tmp1, tmp2);
                    n.normalize();
                    Support(n, simplex[3]);
                    tmp.set(n).negate();
                    Support(tmp, simplex[4]);
                    order = 4;
                    return true;
                case 3:
                case 4:
                    return true;
            }
            return false;
        }
    }

    private static final int[] mod3 = new int[]{0, 1, 2, 0, 1};
    private static final int[][] tetrahedron_fidx = new int[][]{{2, 1, 0}, {3, 0, 1}, {3, 1, 2}, {3, 2, 0}};
    private static final int[][] tetrahedron_eidx = new int[][]{{0, 0, 2, 1}, {0, 1, 1, 1}, {0, 2, 3, 1}, {1, 0, 3, 2}, {2, 0, 1, 2}, {3, 0, 2, 2}};
    private static final int[][] hexahedron_fidx = new int[][]{{2, 0, 4}, {4, 1, 2}, {1, 4, 0}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    private static final int[][] hexahedron_eidx = new int[][]{{0, 0, 4, 0}, {0, 1, 2, 1}, {0, 2, 1, 2}, {1, 1, 5, 2}, {1, 0, 2, 0}, {2, 2, 3, 2}, {3, 1, 5, 0}, {3, 0, 4, 2}, {5, 1, 4, 1}};

    public static class Face {
        public final Mkv[] v = new Mkv[3];
        public final Face[] f = new Face[3];
        public final int[] e = new int[3];
        public final Vector3 n = new Vector3();
        public float d;
        public int mark;
        public Face prev;
        public Face next;
    }

    protected static class EPA {
        public GJK gjk;
        public Face root;
        public int nfaces;
        public int iterations;
        public final Vector3[][] features = new Vector3[2][3];
        public final Vector3[] nearest = new Vector3[]{new Vector3(), new Vector3()};
        public final Vector3 normal = new Vector3();
        public float depth;
        public boolean failed;

        {
            for (var i = 0; i < features.length; i++) {
                for (var j = 0; j < features[i].length; j++) {
                    features[i][j] = new Vector3();
                }
            }
        }

        public EPA(GJK pgjk) {
            gjk = pgjk;
        }

        public Vector3 GetCoordinates(Face face, Vector3 out) {
            var tmp = new Vector3();
            var tmp1 = new Vector3();
            var tmp2 = new Vector3();
            var o = new Vector3();
            float s = -face.d;
            o.set(face.n).mul(s);
            var a = new Float[3];
            tmp1.set(face.v[0].w).sub(o);
            tmp2.set(face.v[1].w).sub(o);
            tmp.cross(tmp1, tmp2);
            a[0] = tmp.length();
            tmp1.set(face.v[1].w).sub(o);
            tmp2.set(face.v[2].w).sub(o);
            tmp.cross(tmp1, tmp2);
            a[1] = tmp.length();
            tmp1.set(face.v[2].w).sub(o);
            tmp2.set(face.v[0].w).sub(o);
            tmp.cross(tmp1, tmp2);
            a[2] = tmp.length();
            var sm = a[0] + a[1] + a[2];
            out.set(a[1], a[2], a[0]);
            out.mul(1f / (sm > 0f ? sm : 1f));
            return out;
        }

        public Face FindBest() {
            Face bf = null;
            if (root != null) {
                var cf = root;
                var bd = cstInf;
                do {
                    if (cf.d < bd) {
                        bd = cf.d;
                        bf = cf;
                    }
                } while (null != (cf = cf.next));
            }
            return bf;
        }

        public boolean Set(Face f, Mkv a, Mkv b, Mkv c) {
            var tmp1 = new Vector3();
            var tmp2 = new Vector3();
            var tmp3 = new Vector3();
            var nrm = new Vector3();
            tmp1.set(b.w).sub(a.w);
            tmp2.set(c.w).sub(a.w);
            nrm.cross(tmp1, tmp2);
            var len = nrm.length();
            tmp1.cross(a.w, b.w);
            tmp2.cross(b.w, c.w);
            tmp3.cross(c.w, a.w);
            var valid = tmp1.dot(nrm) >= -EPA_inface_eps && tmp2.dot(nrm) >= -EPA_inface_eps && tmp3.dot(nrm) >= -EPA_inface_eps;
            f.v[0] = a;
            f.v[1] = b;
            f.v[2] = c;
            f.mark = 0;
            float s = 1f / (len > 0f ? len : cstInf);
            f.n.set(nrm).mul(s);
            f.d = Math.max(0, -f.n.dot(a.w));
            return valid;
        }

        public Face NewFace(Mkv a, Mkv b, Mkv c) {
            var pf = new Face();
            if (Set(pf, a, b, c)) {
                if (root != null) {
                    root.prev = pf;
                }
                pf.prev = null;
                pf.next = root;
                root = pf;
                ++nfaces;
            } else {
                pf.prev = pf.next = null;
            }
            return pf;
        }

        public void Detach(Face face) {
            if (face.prev != null || face.next != null) {
                --nfaces;
                if (face == root) {
                    root = face.next;
                    root.prev = null;
                } else {
                    if (face.next == null) {
                        face.prev.next = null;
                    } else {
                        Objects.requireNonNull(face.prev).next = face.next;
                        face.next.prev = face.prev;
                    }
                }
                face.prev = face.next = null;
            }
        }

        public void Link(Face f0, int e0, Face f1, int e1) {
            f0.f[e0] = f1;
            f1.e[e1] = e0;
            f1.f[e1] = f0;
            f0.e[e0] = e1;
        }

        public Mkv Support(Vector3 w) {
            var v = new Mkv();
            gjk.Support(w, v);
            return v;
        }

        public int BuildHorizon(int markid, Mkv w, Face f, int e, Face[] cf, Face[] ff) {
            var ne = 0;
            if (f.mark != markid) {
                var e1 = mod3[e + 1];
                if (f.n.dot(w.w) + f.d > 0) {
                    var nf = NewFace(f.v[e1], f.v[e], w);
                    Link(nf, 0, f, e);
                    if (cf[0] != null) {
                        Link(cf[0], 1, nf, 2);
                    } else {
                        ff[0] = nf;
                    }
                    cf[0] = nf;
                    ne = 1;
                } else {
                    var e2 = mod3[e + 2];
                    Detach(f);
                    f.mark = markid;
                    ne += BuildHorizon(markid, w, f.f[e1], f.e[e1], cf, ff);
                    ne += BuildHorizon(markid, w, f.f[e2], f.e[e2], cf, ff);
                }
            }
            return ne;
        }

        public float EvaluatePD() {
            return EvaluatePD(EPA_accuracy);
        }

        public float EvaluatePD(float accuracy) {
            var tmp = new Vector3();
            Face bestface = null;
            var markid = 1;
            depth = -cstInf;
            normal.set(0f, 0f, 0f);
            root = null;
            nfaces = 0;
            iterations = 0;
            failed = false;
            if (gjk.EncloseOrigin()) {
                int[][] pfidx_ptr = null;
                var pfidx_index = 0;
                var nfidx = 0;
                int[][] peidx_ptr = null;
                var peidx_index = 0;
                var neidx = 0;
                var basemkv = new Mkv[5];
                var basefaces = new Face[6];
                switch (gjk.order) {
                    case 3:
                        pfidx_ptr = tetrahedron_fidx;
                        pfidx_index = 0;
                        nfidx = 4;
                        peidx_ptr = tetrahedron_eidx;
                        peidx_index = 0;
                        neidx = 6;
                        break;
                    case 4:
                        pfidx_ptr = hexahedron_fidx;
                        pfidx_index = 0;
                        nfidx = 6;
                        peidx_ptr = hexahedron_eidx;
                        peidx_index = 0;
                        neidx = 9;
                        break;
                }
                int i;
                for (i = 0; i <= gjk.order; ++i) {
                    basemkv[i] = new Mkv();
                    basemkv[i].set(gjk.simplex[i]);
                }
                for (i = 0; i < nfidx; ++i, pfidx_index++) {
                    basefaces[i] = NewFace(basemkv[pfidx_ptr[pfidx_index][0]], basemkv[pfidx_ptr[pfidx_index][1]], basemkv[pfidx_ptr[pfidx_index][2]]);
                }
                for (i = 0; i < neidx; ++i, peidx_index++) {
                    Link(basefaces[peidx_ptr[peidx_index][0]], peidx_ptr[peidx_index][1], basefaces[peidx_ptr[peidx_index][2]], peidx_ptr[peidx_index][3]);
                }
            }
            if (0 == nfaces) {
                return depth;
            }
            for (; iterations < EPA_maxiterations; ++iterations) {
                var bf = FindBest();
                if (bf != null) {
                    tmp.set(bf.n).negate();
                    var w = Support(tmp);
                    var d = bf.n.dot(w.w) + bf.d;
                    bestface = bf;
                    if (d < -accuracy) {
                        var cf = new Face[]{null};
                        var ff = new Face[]{null};
                        var nf = 0;
                        Detach(bf);
                        bf.mark = ++markid;
                        for (var i = 0; i < 3; ++i) {
                            nf += BuildHorizon(markid, w, bf.f[i], bf.e[i], cf, ff);
                        }
                        if (nf <= 2) {
                            break;
                        }
                        Link(cf[0], 1, ff[0], 2);
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }
            if (bestface != null) {
                var b = GetCoordinates(bestface, new Vector3());
                normal.set(bestface.n);
                depth = Math.max(0, bestface.d);
                for (var i = 0; i < 2; ++i) {
                    var s = i != 0 ? -1f : 1f;
                    for (var j = 0; j < 3; ++j) {
                        tmp.set(bestface.v[j].r).mul(s);
                        gjk.LocalSupport(tmp, i, features[i][j]);
                    }
                }
                var tmp1 = new Vector3();
                var tmp2 = new Vector3();
                var tmp3 = new Vector3();
                tmp1.set(features[0][0]).mul(b.x);
                tmp2.set(features[0][1]).mul(b.y);
                tmp3.set(features[0][2]).mul(b.z);
                nearest[0].set(tmp1).add(tmp2).add(tmp3);
                tmp1.set(features[1][0]).mul(b.x);
                tmp2.set(features[1][1]).mul(b.y);
                tmp3.set(features[1][2]).mul(b.z);
                nearest[1].set(tmp1).add(tmp2).add(tmp3);
            } else {
                failed = true;
            }
            return depth;
        }
    }

    private final GJK gjk = new GJK();

    public boolean collide(ConvexShape shape0, Transform wtrs0, ConvexShape shape1, Transform wtrs1, float radialmargin, Results results) {
        results.witnesses[0].set(0f, 0f, 0f);
        results.witnesses[1].set(0f, 0f, 0f);
        results.normal.set(0f, 0f, 0f);
        results.depth = 0;
        results.status = ResultsStatus.Separated;
        results.epa_iterations = 0;
        results.gjk_iterations = 0;
        gjk.init(wtrs0.basis, wtrs0.origin, shape0, wtrs1.basis, wtrs1.origin, shape1, radialmargin + EPA_accuracy);
        try {
            var collide = gjk.SearchOrigin();
            results.gjk_iterations = gjk.iterations + 1;
            if (collide) {
                var epa = new EPA(gjk);
                var pd = epa.EvaluatePD();
                results.epa_iterations = epa.iterations + 1;
                if (pd > 0) {
                    results.status = ResultsStatus.Penetrating;
                    results.normal.set(epa.normal);
                    results.depth = pd;
                    results.witnesses[0].set(epa.nearest[0]);
                    results.witnesses[1].set(epa.nearest[1]);
                    return true;
                } else {
                    if (epa.failed) {
                        results.status = ResultsStatus.EPA_Failed;
                    }
                }
            } else {
                if (gjk.failed) {
                    results.status = ResultsStatus.GJK_Failed;
                }
            }
            return false;
        } finally {
            gjk.destroy();
        }
    }
}
