package com.bulletphysics;

public class IntArrayList {
    private int[] array = new int[16];
    private int size;

    public void add(int value) {
        if (size == array.length) {
            expand();
        }
        array[size++] = value;
    }

    private void expand() {
        var newArray = new int[array.length << 1];
        System.arraycopy(array, 0, newArray, 0, array.length);
        array = newArray;
    }

    public void remove(int index) {
        if (index >= size) {
            throw new IndexOutOfBoundsException();
        }
        System.arraycopy(array, index + 1, array, index, size - index - 1);
        size--;
    }

    public int get(int index) {
        if (index >= size) {
            throw new IndexOutOfBoundsException();
        }
        return array[index];
    }

    public void set(int index, int value) {
        if (index >= size) {
            throw new IndexOutOfBoundsException();
        }
        array[index] = value;
    }

    public int size() {
        return size;
    }
}
