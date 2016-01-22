#!/usr/bin/env python3

import re
import sys

re_malloc = re.compile(r'malloc\(\d+\) -> ([A-Za-z0-9]+) @[A-Za-z0-9]+')
re_calloc = re.compile(r'calloc\(\d+, \d+\) -> ([A-Za-z0-9]+) @[A-Za-z0-9]+')
re_memalign = re.compile(r'memalign\(\d+, \d+\) -> ([A-Za-z0-9]+) @[A-Za-z0-9]+')
re_realloc = re.compile(r'realloc\(([A-Za-z0-9]+), \d+\) -> ([A-Za-z0-9]+) @[A-Za-z0-9]+')
re_free = re.compile(r'free\(([A-Za-z0-9]+)\) @[A-Za-z0-9]+')

allocations = {
}

lnum = 0
for l in sys.stdin.readlines():
    lnum += 1
    l = l.strip()
    m = re_malloc.match(l)
    if not m:
        m = re_calloc.match(l)
    if not m:
        m = re_memalign.match(l)
    if m:
        addr = m.group(1)
        if addr in allocations:
            print('ERROR: %d: Duplicate allocation at %s' % (lnum, addr))
        allocations[addr] = [(lnum, l)]
        continue
    m = re_realloc.match(l)
    if m:
        addr_old = m.group(1)
        addr_new = m.group(2)
        if addr_new != addr_old and addr_new in allocations:
            print('ERROR: %d: Duplicate (re-)allocation at %s' % (lnum, addr_new))
        if not addr_old in allocations:
            if addr_old == '0000000000000000':
                old_val = []
            else:
                print('ERROR: %d: Reallocation %s missing' % (lnum, addr_old))
                old_val = [(-1, '???')]
        else:
            old_val = allocations[addr_old]
            del allocations[addr_old]
        new_val = old_val
        new_val.append((lnum, l))
        allocations[addr_new] = new_val
        continue
    m = re_free.match(l)
    if m:
        addr = m.group(1)
        if addr not in allocations:
            print('ERROR: %d: Missing allocation at %s' % (lnum, addr))
        else:
            del allocations[addr]
        continue

for (alloc, hist) in allocations.items():
    print(alloc)
    for ent in hist:
        print('   ', ent)

