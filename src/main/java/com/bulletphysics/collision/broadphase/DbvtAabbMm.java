package com.bulletphysics.collision.broadphase;

import javax.vecmath.Vector3;

public class DbvtAabbMm {
    private final Vector3 mi = new Vector3();
    private final Vector3 mx = new Vector3();

    public DbvtAabbMm() {
    }

    public void set(DbvtAabbMm o) {
        mi.set(o.mi);
        mx.set(o.mx);
    }

    public static void swap(DbvtAabbMm p1, DbvtAabbMm p2) {
        var tmp = new Vector3();
        tmp.set(p1.mi);
        p1.mi.set(p2.mi);
        p2.mi.set(tmp);
        tmp.set(p1.mx);
        p1.mx.set(p2.mx);
        p2.mx.set(tmp);
    }

    public Vector3 Center(Vector3 out) {
        return out.set(mi).add(mx).mul(0.5f);
    }

    public static DbvtAabbMm FromMM(Vector3 mi, Vector3 mx, DbvtAabbMm out) {
        out.mi.set(mi);
        out.mx.set(mx);
        return out;
    }

    public void Expand(Vector3 e) {
        mi.sub(e);
        mx.add(e);
    }

    public void SignedExpand(Vector3 e) {
        if (e.x > 0) {
            mx.x += e.x;
        } else {
            mi.x += e.x;
        }
        if (e.y > 0) {
            mx.y += e.y;
        } else {
            mi.y += e.y;
        }
        if (e.z > 0) {
            mx.z += e.z;
        } else {
            mi.z += e.z;
        }
    }

    public boolean Contain(DbvtAabbMm a) {
        return mi.x <= a.mi.x && mi.y <= a.mi.y && mi.z <= a.mi.z && mx.x >= a.mx.x && mx.y >= a.mx.y && mx.z >= a.mx.z;
    }

    public static boolean Intersect(DbvtAabbMm a, DbvtAabbMm b) {
        return a.mi.x <= b.mx.x && a.mx.x >= b.mi.x && a.mi.y <= b.mx.y && a.mx.y >= b.mi.y && a.mi.z <= b.mx.z && a.mx.z >= b.mi.z;
    }

    public static float Proximity(DbvtAabbMm a, DbvtAabbMm b) {
        var d = new Vector3();
        var tmp = new Vector3();
        d.set(a.mi).add(a.mx);
        tmp.set(b.mi).add(b.mx);
        d.sub(tmp);
        return Math.abs(d.x) + Math.abs(d.y) + Math.abs(d.z);
    }

    public static void Merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm r) {
        for (var i = 0; i < 3; i++) {
            r.mi.setCoord(i, Math.min(a.mi.getCoord(i), b.mi.getCoord(i)));
            r.mx.setCoord(i, Math.max(a.mx.getCoord(i), b.mx.getCoord(i)));
        }
    }

    public static boolean NotEqual(DbvtAabbMm a, DbvtAabbMm b) {
        return a.mi.x != b.mi.x || a.mi.y != b.mi.y || a.mi.z != b.mi.z || a.mx.x != b.mx.x || a.mx.y != b.mx.y || a.mx.z != b.mx.z;
    }
}
