package com.bulletphysics.collision.broadphase;

public abstract class OverlappingPairCallback {
    public abstract BroadPhasePair addOverlappingPair(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1);

    public abstract Object removeOverlappingPair(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1, Dispatcher dispatcher);
}
