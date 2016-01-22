#!/usr/bin/env python3

import re
import sys

re_malloc = re.compile(r'malloc\((\d+)\) -> ([A-Za-z0-9]+) @[A-Za-z0-9]+')
re_calloc = re.compile(r'calloc\((\d+), (\d+)\) -> ([A-Za-z0-9]+) @[A-Za-z0-9]+')
re_memalign = re.compile(r'memalign\(\d+, (\d+)\) -> ([A-Za-z0-9]+) @[A-Za-z0-9]+')
re_realloc = re.compile(r'realloc\(([A-Za-z0-9]+), (\d+)\) -> ([A-Za-z0-9]+) @[A-Za-z0-9]+')
re_free = re.compile(r'free\(([A-Za-z0-9]+)\) @[A-Za-z0-9]+')

allocations = {
}

def validate(lnum):
    # addr, size
    l = [(addr, allocations[addr][-1][2]) for addr in sorted(allocations.keys())]
    #print(repr(l))
    for i in range(len(l) - 1):
        if l[i][0] + l[i][1] > l[i + 1][0]:
            print("%d: ERROR: Block 0x%x+%d overlaps block 0x%x+%d" % tuple((lnum,) + l[i] + l[i + 1]))
    if lnum == 3973:
        for (addr, size) in l:
            print("0x%x +%d" % (addr, size))
        sys.exit(0)

lnum = 0
for l in sys.stdin.readlines():
    lnum += 1
    validate(lnum)
    l = l.strip()
    addr = None
    m = re_malloc.search(l)
    if m:
        size = int(m.group(1))
        addr = int(m.group(2), 16)
    if not m:
        m = re_calloc.search(l)
        if m:
            size = int(m.group(1))
            size *= int(m.group(2))
            addr = int(m.group(3), 16)
    if not m:
        m = re_memalign.search(l)
        if m:
            size = int(m.group(1))
            addr = int(m.group(2), 16)
    if addr is not None:
        if addr in allocations:
            print('ERROR: %d: Duplicate allocation at 0x%x' % (lnum, addr))
        allocations[addr] = [(lnum, l, size)]
        continue
    m = re_realloc.search(l)
    if m:
        addr_old = int(m.group(1), 16)
        size = int(m.group(2))
        addr_new = int(m.group(3), 16)
        if addr_new != addr_old and addr_new in allocations:
            print('ERROR: %d: Duplicate (re-)allocation at %s' % (lnum, addr_new))
        if not addr_old in allocations:
            if addr_old == 0:
                old_val = []
            else:
                print('ERROR: %d: Reallocation 0x%x missing' % (lnum, addr_old))
                old_val = [(-1, '???', 0)]
        else:
            old_val = allocations[addr_old]
            del allocations[addr_old]
        new_val = old_val
        new_val.append((lnum, l, size))
        allocations[addr_new] = new_val
        continue
    m = re_free.search(l)
    if m:
        addr = int(m.group(1), 16)
        if addr not in allocations:
            if addr != 0:
                print('ERROR: %d: Missing allocation at 0x%x' % (lnum, addr))
        else:
            del allocations[addr]
        continue

for (alloc, hist) in allocations.items():
    print("0x%x" % alloc)
    for ent in hist:
        print('   ', ent)

