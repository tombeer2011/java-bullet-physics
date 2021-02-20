package com.bulletphysics.collision.broadphase;

import javax.vecmath.Vector3;

public class DbvtBroadPhase extends BroadPhaseInterface {
    public static final float DBVT_BP_MARGIN = 0.05f;
    public static final int STAGECOUNT = 2;
    public final Dbvt[] sets = new Dbvt[2];
    public final DbvtProxy[] stageRoots = new DbvtProxy[STAGECOUNT + 1];
    public final OverlappingPairCache paircache;
    public final float predictedframes;
    public int stageCurrent;
    public final int fupdates;
    public final int dupdates;
    public int pid;
    public int gid;
    public final boolean releasepaircache;

    public DbvtBroadPhase() {
        this(null);
    }

    public DbvtBroadPhase(OverlappingPairCache paircache) {
        sets[0] = new Dbvt();
        sets[1] = new Dbvt();
        releasepaircache = paircache == null;
        predictedframes = 2;
        stageCurrent = 0;
        fupdates = 1;
        dupdates = 1;
        this.paircache = paircache != null ? paircache : new HashedOverlappingPairCache();
        gid = 0;
        pid = 0;
        for (var i = 0; i <= STAGECOUNT; i++) {
            stageRoots[i] = null;
        }
    }

    public void collide(Dispatcher dispatcher) {
        sets[0].optimizeIncremental(1 + sets[0].leaves * dupdates / 100);
        sets[1].optimizeIncremental(1 + sets[1].leaves * fupdates / 100);
        stageCurrent = (stageCurrent + 1) % STAGECOUNT;
        var current = stageRoots[stageCurrent];
        if (current != null) {
            var collider = new DbvtTreeCollider(this);
            do {
                var next = current.links[1];
                stageRoots[current.stage] = listremove(current, stageRoots[current.stage]);
                stageRoots[STAGECOUNT] = listappend(current, stageRoots[STAGECOUNT]);
                Dbvt.collideTT(sets[1].root, current.leaf, collider);
                sets[0].remove(current.leaf);
                current.leaf = sets[1].insert(current.aabb, current);
                current.stage = STAGECOUNT;
                current = next;
            } while (current != null);
        }
        var collider = new DbvtTreeCollider(this);
        Dbvt.collideTT(sets[0].root, sets[1].root, collider);
        Dbvt.collideTT(sets[0].root, sets[0].root, collider);
        var pairs = paircache.getOverlappingPairArray();
        if (pairs.size() > 0) {
            for (int i = 0, ni = pairs.size(); i < ni; i++) {
                var p = pairs.get(i);
                var pa = (DbvtProxy) p.pProxy0;
                var pb = (DbvtProxy) p.pProxy1;
                if (!DbvtAabbMm.Intersect(pa.aabb, pb.aabb)) {
                    if (pa.hashCode() > pb.hashCode()) {
                        var tmp = pa;
                        pa = pb;
                        pb = tmp;
                    }
                    paircache.removeOverlappingPair(pa, pb, dispatcher);
                    ni--;
                    i--;
                }
            }
        }
        pid++;
    }

    private static DbvtProxy listappend(DbvtProxy item, DbvtProxy list) {
        item.links[0] = null;
        item.links[1] = list;
        if (list != null) {
            list.links[0] = item;
        }
        list = item;
        return list;
    }

    private static DbvtProxy listremove(DbvtProxy item, DbvtProxy list) {
        if (item.links[0] != null) {
            item.links[0].links[1] = item.links[1];
        } else {
            list = item.links[1];
        }
        if (item.links[1] != null) {
            item.links[1].links[0] = item.links[0];
        }
        return list;
    }

    public BroadPhaseProxy createProxy(Vector3 aabbMin, Vector3 aabbMax, Object userPtr, short collisionFilterGroup, short collisionFilterMask) {
        var proxy = new DbvtProxy(userPtr, collisionFilterGroup, collisionFilterMask);
        DbvtAabbMm.FromMM(aabbMin, aabbMax, proxy.aabb);
        proxy.leaf = sets[0].insert(proxy.aabb, proxy);
        proxy.stage = stageCurrent;
        proxy.uniqueId = ++gid;
        stageRoots[stageCurrent] = listappend(proxy, stageRoots[stageCurrent]);
        return proxy;
    }

    public void setAabb(BroadPhaseProxy absproxy, Vector3 aabbMin, Vector3 aabbMax) {
        var proxy = (DbvtProxy) absproxy;
        var aabb = DbvtAabbMm.FromMM(aabbMin, aabbMax, new DbvtAabbMm());
        if (proxy.stage == STAGECOUNT) {
            sets[1].remove(proxy.leaf);
            proxy.leaf = sets[0].insert(aabb, proxy);
        } else {
            if (DbvtAabbMm.Intersect(proxy.leaf.volume, aabb)) {
                var delta = new Vector3();
                delta.set(aabbMin).add(aabbMax);
                delta.mul(0.5f);
                delta.sub(proxy.aabb.Center(new Vector3()));
                delta.mul(predictedframes);
                sets[0].update(proxy.leaf, aabb, delta, DBVT_BP_MARGIN);
            } else {
                sets[0].update(proxy.leaf, aabb);
            }
        }
        stageRoots[proxy.stage] = listremove(proxy, stageRoots[proxy.stage]);
        proxy.aabb.set(aabb);
        proxy.stage = stageCurrent;
        stageRoots[stageCurrent] = listappend(proxy, stageRoots[stageCurrent]);
    }

    public void calculateOverlappingPairs(Dispatcher dispatcher) {
        collide(dispatcher);
    }

    public OverlappingPairCache getOverlappingPairCache() {
        return paircache;
    }
}
