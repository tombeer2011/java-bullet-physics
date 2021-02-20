package com.bulletphysics.collision.broadphase;

import java.util.List;

public abstract class OverlappingPairCache extends OverlappingPairCallback {
    public abstract List<BroadPhasePair> getOverlappingPairArray();

    public abstract void cleanOverlappingPair(BroadPhasePair pair, Dispatcher dispatcher);

    public abstract void cleanProxyFromPairs(BroadPhaseProxy proxy, Dispatcher dispatcher);

    public abstract void processAllOverlappingPairs(OverlapCallback callback, Dispatcher dispatcher);

    public abstract BroadPhasePair findPair(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1);
}
