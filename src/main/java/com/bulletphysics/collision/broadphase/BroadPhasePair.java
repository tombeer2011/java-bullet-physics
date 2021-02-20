package com.bulletphysics.collision.broadphase;

public class BroadPhasePair {
    public BroadPhaseProxy pProxy0;
    public BroadPhaseProxy pProxy1;
    public CollisionAlgorithm algorithm;
    public Object userInfo;

    public BroadPhasePair(BroadPhaseProxy pProxy0, BroadPhaseProxy pProxy1) {
        this.pProxy0 = pProxy0;
        this.pProxy1 = pProxy1;
        algorithm = null;
        userInfo = null;
    }

    public void set(BroadPhasePair p) {
        pProxy0 = p.pProxy0;
        pProxy1 = p.pProxy1;
        algorithm = p.algorithm;
        userInfo = p.userInfo;
    }

    public boolean equals(BroadPhasePair p) {
        return pProxy0 == p.pProxy0 && pProxy1 == p.pProxy1;
    }
}
