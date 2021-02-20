package com.bulletphysics.collision.broadphase;

public abstract class OverlapCallback {
    public abstract boolean processOverlap(BroadPhasePair pair);
}
