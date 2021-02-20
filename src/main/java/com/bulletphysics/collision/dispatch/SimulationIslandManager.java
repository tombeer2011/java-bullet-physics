package com.bulletphysics.collision.dispatch;

import com.bulletphysics.MiscUtil;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.narrowphase.PersistentManifold;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

public class SimulationIslandManager {
    private final UnionFind unionFind = new UnionFind();
    private final ArrayList<PersistentManifold> islandManifold = new ArrayList<>();
    private final ArrayList<CollisionObject> islandBodies = new ArrayList<>();

    public void initUnionFind(int n) {
        unionFind.reset(n);
    }

    public UnionFind getUnionFind() {
        return unionFind;
    }

    public void findUnions(CollisionWorld colWorld) {
        var pairPtr = colWorld.getPairCache().getOverlappingPairArray();
        for (var i = 0; i < pairPtr.size(); i++) {
            var collisionPair = pairPtr.get(i);
            var colObj0 = (CollisionObject) collisionPair.pProxy0.clientObject;
            var colObj1 = (CollisionObject) collisionPair.pProxy1.clientObject;
            if (colObj0 != null && colObj0.mergesSimulationIslands() && colObj1 != null && colObj1.mergesSimulationIslands()) {
                unionFind.unite(colObj0.getIslandTag(), colObj1.getIslandTag());
            }
        }
    }

    public void updateActivationState(CollisionWorld colWorld) {
        initUnionFind(colWorld.getCollisionObjectArray().size());
        var index = 0;
        int i;
        for (i = 0; i < colWorld.getCollisionObjectArray().size(); i++) {
            var collisionObject = colWorld.getCollisionObjectArray().get(i);
            collisionObject.setIslandTag(index);
            collisionObject.setCompanionId(-1);
            collisionObject.setHitFraction(1f);
            index++;
        }
        findUnions(colWorld);
    }

    public void storeIslandActivationState(CollisionWorld colWorld) {
        var index = 0;
        int i;
        for (i = 0; i < colWorld.getCollisionObjectArray().size(); i++) {
            var collisionObject = colWorld.getCollisionObjectArray().get(i);
            if (!collisionObject.isStaticOrKinematicObject()) {
                collisionObject.setIslandTag(unionFind.find(index));
                collisionObject.setCompanionId(-1);
            } else {
                collisionObject.setIslandTag(-1);
                collisionObject.setCompanionId(-2);
            }
            index++;
        }
    }

    private static int getIslandId(PersistentManifold lhs) {
        int islandId;
        var rcolObj0 = (CollisionObject) lhs.getBody0();
        var rcolObj1 = (CollisionObject) lhs.getBody1();
        islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
        return islandId;
    }

    public void buildIslands(Dispatcher dispatcher, List<CollisionObject> collisionObjects) {
        islandManifold.clear();
        getUnionFind().sortIslands();
        var numElem = getUnionFind().getNumElements();
        var endIslandIndex = 1;
        int startIslandIndex;
        for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
            var islandId = getUnionFind().getElement(startIslandIndex).id;
            endIslandIndex = startIslandIndex + 1;
            while (endIslandIndex < numElem && getUnionFind().getElement(endIslandIndex).id == islandId) {
                endIslandIndex++;
            }
            var allSleeping = true;
            int idx;
            for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
                var i = getUnionFind().getElement(idx).sz;
                var colObj0 = collisionObjects.get(i);

                if (colObj0.getIslandTag() == islandId) {
                    if (colObj0.getActivationState() == CollisionObject.ACTIVE_TAG) {
                        allSleeping = false;
                    }
                    if (colObj0.getActivationState() == CollisionObject.DISABLE_DEACTIVATION) {
                        allSleeping = false;
                    }
                }
            }
            if (allSleeping) {
                for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
                    var i = getUnionFind().getElement(idx).sz;
                    var colObj0 = collisionObjects.get(i);

                    if (colObj0.getIslandTag() == islandId) {
                        colObj0.setActivationState(CollisionObject.ISLAND_SLEEPING);
                    }
                }
            } else {
                for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
                    var i = getUnionFind().getElement(idx).sz;
                    var colObj0 = collisionObjects.get(i);

                    if (colObj0.getIslandTag() == islandId) {
                        if (colObj0.getActivationState() == CollisionObject.ISLAND_SLEEPING) {
                            colObj0.setActivationState(CollisionObject.WANTS_DEACTIVATION);
                        }
                    }
                }
            }
        }
        int i;
        var maxNumManifolds = dispatcher.getNumManifolds();
        for (i = 0; i < maxNumManifolds; i++) {
            var manifold = dispatcher.getManifoldByIndexInternal(i);
            var colObj0 = (CollisionObject) manifold.getBody0();
            var colObj1 = (CollisionObject) manifold.getBody1();
            if (colObj0 != null && colObj0.getActivationState() != CollisionObject.ISLAND_SLEEPING || colObj1 != null && colObj1.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                if (Objects.requireNonNull(colObj0).isKinematicObject() && colObj0.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                    colObj1.activate();
                }
                if (colObj1.isKinematicObject() && colObj1.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                    colObj0.activate();
                }
                if (dispatcher.needsResponse(colObj0, colObj1)) {
                    islandManifold.add(manifold);
                }
            }
        }
    }

    public void buildAndProcessIslands(Dispatcher dispatcher, List<CollisionObject> collisionObjects, IslandCallback callback) {
        buildIslands(dispatcher, collisionObjects);
        var endIslandIndex = 1;
        int startIslandIndex;
        var numElem = getUnionFind().getNumElements();
        var numManifolds = islandManifold.size();
        MiscUtil.quickSort(islandManifold, persistentManifoldComparator);
        var startManifoldIndex = 0;
        var endManifoldIndex = 1;
        for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
            var islandId = getUnionFind().getElement(startIslandIndex).id;
            var islandSleeping = false;
            for (endIslandIndex = startIslandIndex; endIslandIndex < numElem && getUnionFind().getElement(endIslandIndex).id == islandId; endIslandIndex++) {
                var i = getUnionFind().getElement(endIslandIndex).sz;
                var colObj0 = collisionObjects.get(i);
                islandBodies.add(colObj0);
                if (!colObj0.isActive()) {
                    islandSleeping = true;
                }
            }
            var numIslandManifolds = 0;
            var startManifold_idx = -1;
            if (startManifoldIndex < numManifolds) {
                var curIslandId = getIslandId(islandManifold.get(startManifoldIndex));
                if (curIslandId == islandId) {
                    startManifold_idx = startManifoldIndex;
                    endManifoldIndex = startManifoldIndex + 1;
                    while (endManifoldIndex < numManifolds && islandId == getIslandId(islandManifold.get(endManifoldIndex))) {
                        endManifoldIndex++;
                    }
                    numIslandManifolds = endManifoldIndex - startManifoldIndex;
                }
            }
            if (!islandSleeping) {
                callback.processIsland(islandBodies, islandBodies.size(), islandManifold, startManifold_idx, numIslandManifolds, islandId);
            }
            if (numIslandManifolds != 0) {
                startManifoldIndex = endManifoldIndex;
            }
            islandBodies.clear();
        }
    }

    public static abstract class IslandCallback {
        public abstract void processIsland(List<CollisionObject> bodies, int numBodies, List<PersistentManifold> manifolds, int manifolds_offset, int numManifolds, int islandId);
    }

    private static final Comparator<PersistentManifold> persistentManifoldComparator = (lhs, rhs) -> getIslandId(lhs) < getIslandId(rhs) ? -1 : +1;
}
