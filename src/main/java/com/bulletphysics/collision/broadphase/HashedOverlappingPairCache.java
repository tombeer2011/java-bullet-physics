package com.bulletphysics.collision.broadphase;

import com.bulletphysics.ArrayList;
import com.bulletphysics.IntArrayList;
import com.bulletphysics.MiscUtil;

public class HashedOverlappingPairCache extends OverlappingPairCache {
    private static final int NULL_PAIR = 0xffffffff;
    private final ArrayList<BroadPhasePair> overlappingPairArray = new ArrayList<>();
    private final IntArrayList hashTable = new IntArrayList();
    private final IntArrayList next = new IntArrayList();
    protected OverlappingPairCallback ghostPairCallback;

    public HashedOverlappingPairCache() {
        growTables();
    }

    public BroadPhasePair addOverlappingPair(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1) {
        if (!needsBroadphaseCollision(proxy0, proxy1)) {
            return null;
        }
        return internalAddPair(proxy0, proxy1);
    }

    public Object removeOverlappingPair(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1, Dispatcher dispatcher) {
        if (proxy0.getUid() > proxy1.getUid()) {
            var tmp = proxy0;
            proxy0 = proxy1;
            proxy1 = tmp;
        }
        var proxyId1 = proxy0.getUid();
        var proxyId2 = proxy1.getUid();
        var hash = getHash(proxyId1, proxyId2) & overlappingPairArray.size() - 1;
        var pair = internalFindPair(proxy0, proxy1, hash);
        if (pair == null) {
            return null;
        }
        cleanOverlappingPair(pair, dispatcher);
        var userData = pair.userInfo;


        var pairIndex = overlappingPairArray.indexOf(pair);


        var index = hashTable.get(hash);

        var previous = NULL_PAIR;
        while (index != pairIndex) {
            previous = index;
            index = next.get(index);
        }
        if (previous != NULL_PAIR) {

            next.set(previous, next.get(pairIndex));
        } else {
            hashTable.set(hash, next.get(pairIndex));
        }
        var lastPairIndex = overlappingPairArray.size() - 1;
        if (ghostPairCallback != null) {
            ghostPairCallback.removeOverlappingPair(proxy0, proxy1, dispatcher);
        }
        if (lastPairIndex == pairIndex) {
            overlappingPairArray.remove(overlappingPairArray.size() - 1);
            return userData;
        }
        var last = overlappingPairArray.get(lastPairIndex);
        var lastHash = getHash(last.pProxy0.getUid(), last.pProxy1.getUid()) & overlappingPairArray.capacity() - 1;
        index = hashTable.get(lastHash);

        previous = NULL_PAIR;
        while (index != lastPairIndex) {
            previous = index;
            index = next.get(index);
        }
        if (previous != NULL_PAIR) {

            next.set(previous, next.get(lastPairIndex));
        } else {
            hashTable.set(lastHash, next.get(lastPairIndex));
        }
        overlappingPairArray.get(pairIndex).set(overlappingPairArray.get(lastPairIndex));
        next.set(pairIndex, hashTable.get(lastHash));
        hashTable.set(lastHash, pairIndex);
        overlappingPairArray.remove(overlappingPairArray.size() - 1);
        return userData;
    }

    public boolean needsBroadphaseCollision(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1) {
        var collides = (proxy0.collisionFilterGroup & proxy1.collisionFilterMask) != 0;
        collides = collides && (proxy1.collisionFilterGroup & proxy0.collisionFilterMask) != 0;
        return collides;
    }

    @Override
    public void processAllOverlappingPairs(OverlapCallback callback, Dispatcher dispatcher) {
        for (var i = 0; i < overlappingPairArray.size(); ) {
            var pair = overlappingPairArray.get(i);
            if (callback.processOverlap(pair)) {
                removeOverlappingPair(pair.pProxy0, pair.pProxy1, dispatcher);
            } else {
                i++;
            }
        }
    }

    @Override
    public void cleanProxyFromPairs(BroadPhaseProxy proxy, Dispatcher dispatcher) {
        processAllOverlappingPairs(new CleanPairCallback(proxy, this, dispatcher), dispatcher);
    }

    @Override
    public ArrayList<BroadPhasePair> getOverlappingPairArray() {
        return overlappingPairArray;
    }

    @Override
    public void cleanOverlappingPair(BroadPhasePair pair, Dispatcher dispatcher) {
        if (pair.algorithm != null) {
            dispatcher.freeCollisionAlgorithm(pair.algorithm);
            pair.algorithm = null;
        }
    }

    @Override
    public BroadPhasePair findPair(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1) {
        if (proxy0.getUid() > proxy1.getUid()) {
            proxy0 = proxy1;
            proxy1 = proxy0;
        }
        var proxyId1 = proxy0.getUid();
        var proxyId2 = proxy1.getUid();
        var hash = getHash(proxyId1, proxyId2) & overlappingPairArray.capacity() - 1;
        if (hash >= hashTable.size()) {
            return null;
        }
        var index = hashTable.get(hash);
        while (index != NULL_PAIR && equalsPair(overlappingPairArray.get(index), proxyId1, proxyId2)) {
            index = next.get(index);
        }
        if (index == NULL_PAIR) {
            return null;
        }

        return overlappingPairArray.get(index);
    }

    private BroadPhasePair internalAddPair(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1) {
        if (proxy0.getUid() > proxy1.getUid()) {
            var tmp = proxy0;
            proxy0 = proxy1;
            proxy1 = tmp;
        }
        var proxyId1 = proxy0.getUid();
        var proxyId2 = proxy1.getUid();
        var hash = getHash(proxyId1, proxyId2) & overlappingPairArray.capacity() - 1;
        var pair = internalFindPair(proxy0, proxy1, hash);
        if (pair != null) {
            return pair;
        }
        var count = overlappingPairArray.size();
        var oldCapacity = overlappingPairArray.capacity();
        overlappingPairArray.add(null);
        if (ghostPairCallback != null) {
            ghostPairCallback.addOverlappingPair(proxy0, proxy1);
        }
        var newCapacity = overlappingPairArray.capacity();
        if (oldCapacity < newCapacity) {
            growTables();
            hash = getHash(proxyId1, proxyId2) & overlappingPairArray.capacity() - 1;
        }
        pair = new BroadPhasePair(proxy0, proxy1);
        pair.algorithm = null;
        pair.userInfo = null;
        overlappingPairArray.set(overlappingPairArray.size() - 1, pair);
        next.set(count, hashTable.get(hash));
        hashTable.set(hash, count);
        return pair;
    }

    private void growTables() {
        var newCapacity = overlappingPairArray.capacity();
        if (hashTable.size() < newCapacity) {
            var curHashtableSize = hashTable.size();
            MiscUtil.resize(hashTable, newCapacity, 0);
            MiscUtil.resize(next, newCapacity, 0);
            for (var i = 0; i < newCapacity; ++i) {
                hashTable.set(i, NULL_PAIR);
            }
            for (var i = 0; i < newCapacity; ++i) {
                next.set(i, NULL_PAIR);
            }
            for (var i = 0; i < curHashtableSize; i++) {
                var pair = overlappingPairArray.get(i);
                var proxyId1 = pair.pProxy0.getUid();
                var proxyId2 = pair.pProxy1.getUid();
                var hashValue = getHash(proxyId1, proxyId2) & overlappingPairArray.capacity() - 1;
                next.set(i, hashTable.get(hashValue));
                hashTable.set(hashValue, i);
            }
        }
    }

    private boolean equalsPair(BroadPhasePair pair, int proxyId1, int proxyId2) {
        return pair.pProxy0.getUid() != proxyId1 || pair.pProxy1.getUid() != proxyId2;
    }

    private int getHash(int proxyId1, int proxyId2) {
        var key = proxyId1 | proxyId2 << 16;
        key += ~(key << 15);
        key ^= key >>> 10;
        key += key << 3;
        key ^= key >>> 6;
        key += ~(key << 11);
        key ^= key >>> 16;
        return key;
    }

    private BroadPhasePair internalFindPair(BroadPhaseProxy proxy0, BroadPhaseProxy proxy1, int hash) {
        var proxyId1 = proxy0.getUid();
        var proxyId2 = proxy1.getUid();
        var index = hashTable.get(hash);
        while (index != NULL_PAIR && equalsPair(overlappingPairArray.get(index), proxyId1, proxyId2)) {
            index = next.get(index);
        }
        if (index == NULL_PAIR) {
            return null;
        }

        return overlappingPairArray.get(index);
    }

    private static class CleanPairCallback extends OverlapCallback {
        private final BroadPhaseProxy cleanProxy;
        private final OverlappingPairCache pairCache;
        private final Dispatcher dispatcher;

        public CleanPairCallback(BroadPhaseProxy cleanProxy, OverlappingPairCache pairCache, Dispatcher dispatcher) {
            this.cleanProxy = cleanProxy;
            this.pairCache = pairCache;
            this.dispatcher = dispatcher;
        }

        public boolean processOverlap(BroadPhasePair pair) {
            if (pair.pProxy0 == cleanProxy || pair.pProxy1 == cleanProxy) {
                pairCache.cleanOverlappingPair(pair, dispatcher);
            }
            return false;
        }
    }
}
