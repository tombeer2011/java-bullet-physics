package com.bulletphysics;

import java.util.AbstractList;
import java.util.Objects;
import java.util.RandomAccess;
import java.util.stream.IntStream;

public final class ArrayList<T> extends AbstractList<T> implements RandomAccess {
    private T[] array;
    private int size;

    @SuppressWarnings("unchecked")
    public ArrayList(int initialCapacity) {
        array = (T[]) new Object[initialCapacity];
    }

    public ArrayList() {
        this(16);
    }

    public int size() {
        return size;
    }

    public int capacity() {
        return array.length;
    }

    @Override
    public boolean add(T value) {
        if (size == array.length) expand();
        array[size++] = value;
        return true;
    }

    @Override
    public void add(int index, T value) {
        if (size == array.length) expand();
        var num = size - index;
        if (num > 0) System.arraycopy(array, index, array, index + 1, num);
        array[index] = value;
        size++;
    }

    @Override
    public T remove(int index) {
        if (index < 0 || index >= size) throw new IndexOutOfBoundsException();
        var prev = array[index];
        System.arraycopy(array, index + 1, array, index, size - index - 1);
        array[size - 1] = null;
        size--;
        return prev;
    }

    public T get(int index) {
        if (index >= size) throw new IndexOutOfBoundsException();
        return array[index];
    }

    @Override
    public T set(int index, T value) {
        if (index >= size) throw new IndexOutOfBoundsException();
        var old = array[index];
        array[index] = value;
        return old;
    }

    @Override
    public void clear() {
        size = 0;
    }

    @Override
    public int indexOf(Object o) {
        var _size = size;
        var _array = array;
        return IntStream.range(0, _size).filter(i -> Objects.equals(o, _array[i])).findFirst().orElse(-1);
    }

    @SuppressWarnings("unchecked")
    private void expand() {
        var newArray = (T[]) new Object[array.length << 1];
        System.arraycopy(array, 0, newArray, 0, array.length);
        array = newArray;
    }
}
