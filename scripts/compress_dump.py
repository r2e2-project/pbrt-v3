#!/usr/bin/env python3

import re
import os
import sys
import shutil
import tempfile
import functools
import subprocess as sub
import multiprocessing as mp


def compress_one_treelet(name, src_dir, tmp_dir, dst_dir):
    src_name = os.path.join(src_dir, name)
    tmp_name = os.path.join(tmp_dir, name)
    dst_name = os.path.join(dst_dir, name)

    src_size = os.stat(src_name).st_size
    sub.check_call(["lz4", src_name, tmp_name], stdout=sub.DEVNULL, stderr=sub.DEVNULL)
    tmp_size = os.stat(tmp_name).st_size

    if tmp_size / src_size < 0.66:
        print(f"{name} -> compressed")
        return shutil.move(tmp_name, dst_name)
    else:
        print(f"{name} -> not compressed")
        os.remove(tmp_name)
        return shutil.copy(src_name, dst_name)


def compress(src_dir, dst_dir):
    with tempfile.TemporaryDirectory(dir="/dev/shm/") as tmp_dir:
        treelet_file_pattern = re.compile(r"^T\d+$")
        treelet_files = os.listdir(src_dir)
        other_files = [x for x in treelet_files if not treelet_file_pattern.match(x)]
        treelet_files = [x for x in treelet_files if treelet_file_pattern.match(x)]
        print(
            f"Compressing {len(treelet_files)} treelets using {mp.cpu_count()} cores:"
        )

        with mp.Pool(mp.cpu_count()) as pool:
            f = functools.partial(
                compress_one_treelet, src_dir=src_dir, tmp_dir=tmp_dir, dst_dir=dst_dir
            )

            pool.map(f, treelet_files)

        for f in other_files:
            shutil.copy(os.path.join(src_dir, f), os.path.join(dst_dir, f))

        print("Done.")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} SRC DST")
        sys.exit(1)

    src_dir = sys.argv[1]
    dst_dir = sys.argv[2]

    compress(src_dir, dst_dir)
