package com.bulletphysics.collision.broadphase;

public class BroadPhaseProxy {
    public Object clientObject;
    public short collisionFilterGroup;
    public short collisionFilterMask;
    public Object multiSapParentProxy;
    public int uniqueId;

    public BroadPhaseProxy() {
    }

    public BroadPhaseProxy(Object userPtr, short collisionFilterGroup, short collisionFilterMask) {
        this(userPtr, collisionFilterGroup, collisionFilterMask, null);
    }

    public BroadPhaseProxy(Object userPtr, short collisionFilterGroup, short collisionFilterMask, Object multiSapParentProxy) {
        clientObject = userPtr;
        this.collisionFilterGroup = collisionFilterGroup;
        this.collisionFilterMask = collisionFilterMask;
        this.multiSapParentProxy = multiSapParentProxy;
    }

    public int getUid() {
        return uniqueId;
    }
}
