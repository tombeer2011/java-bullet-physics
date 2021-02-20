package com.bulletphysics;

import java.lang.reflect.InvocationTargetException;
import java.util.Comparator;
import java.util.List;

public class MiscUtil {
    public static void resize(IntArrayList list, int size, int value) {
        while (list.size() < size) {
            list.add(value);
        }
        while (list.size() > size) {
            list.remove(list.size() - 1);
        }
    }

    public static <T> void resize(List<T> list, int size, Class<T> valueCls) {
        try {
            while (list.size() < size) {
                list.add(valueCls != null ? valueCls.getDeclaredConstructor().newInstance() : null);
            }
            while (list.size() > size) {
                list.remove(list.size() - 1);
            }
        } catch (IllegalAccessException | InstantiationException | NoSuchMethodException | InvocationTargetException e) {
            throw new IllegalStateException(e);
        }
    }

    private static <T> void swap(List<T> list, int index0, int index1) {
        var temp = list.get(index0);
        list.set(index0, list.get(index1));
        list.set(index1, temp);
    }

    public static <T> void quickSort(List<T> list, Comparator<T> comparator) {
        if (list.size() > 1) {
            quickSortInternal(list, comparator, 0, list.size() - 1);
        }
    }

    private static <T> void quickSortInternal(List<T> list, Comparator<T> comparator, int lo, int hi) {
        int i = lo, j = hi;
        var x = list.get((lo + hi) / 2);
        do {
            while (comparator.compare(list.get(i), x) < 0) {
                i++;
            }
            while (comparator.compare(x, list.get(j)) < 0) {
                j--;
            }
            if (i <= j) {
                swap(list, i, j);
                i++;
                j--;
            }
        } while (i <= j);
        if (lo < j) {
            quickSortInternal(list, comparator, lo, j);
        }
        if (i < hi) {
            quickSortInternal(list, comparator, i, hi);
        }
    }
}
