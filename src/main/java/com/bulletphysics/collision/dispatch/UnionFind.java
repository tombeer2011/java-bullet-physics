package com.bulletphysics.collision.dispatch;

import com.bulletphysics.ArrayList;
import com.bulletphysics.MiscUtil;

import java.util.Comparator;
import java.util.List;

public class UnionFind {
    private final List<Element> elements = new ArrayList<>();

    public void sortIslands() {
        var numElements = elements.size();
        for (var i = 0; i < numElements; i++) {
            elements.get(i).id = find(i);
            elements.get(i).sz = i;
        }
        MiscUtil.quickSort(elements, elementComparator);
    }

    public void reset(int N) {
        allocate(N);
        for (var i = 0; i < N; i++) {
            elements.get(i).id = i;
            elements.get(i).sz = 1;
        }
    }

    public int getNumElements() {
        return elements.size();
    }

    public Element getElement(int index) {
        return elements.get(index);
    }

    public void allocate(int N) {
        MiscUtil.resize(elements, N, Element.class);
    }

    public void unite(int p, int q) {
        int i = find(p), j = find(q);
        if (i == j) {
            return;
        }
        elements.get(i).id = j;
        elements.get(j).sz += elements.get(i).sz;
    }

    public int find(int x) {
        while (x != elements.get(x).id) {
            elements.get(x).id = elements.get(elements.get(x).id).id;
            x = elements.get(x).id;
        }
        return x;
    }

    public static class Element {
        public int id;
        public int sz;
    }

    private static final Comparator<Element> elementComparator = (o1, o2) -> o1.id < o2.id ? -1 : 1;
}
