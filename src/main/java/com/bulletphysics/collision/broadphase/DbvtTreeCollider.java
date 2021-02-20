package com.bulletphysics.collision.broadphase;

public class DbvtTreeCollider extends Dbvt.ICollide {
    public final DbvtBroadPhase pbp;

    public DbvtTreeCollider(DbvtBroadPhase p) {
        pbp = p;
    }

    @Override
    public void Process(Dbvt.Node na, Dbvt.Node nb) {
        var pa = (DbvtProxy) na.data;
        var pb = (DbvtProxy) nb.data;
        if (DbvtAabbMm.Intersect(pa.aabb, pb.aabb)) {
            if (pa.hashCode() > pb.hashCode()) {
                var tmp = pa;
                pa = pb;
                pb = tmp;
            }
            pbp.paircache.addOverlappingPair(pa, pb);
        }
    }
}
