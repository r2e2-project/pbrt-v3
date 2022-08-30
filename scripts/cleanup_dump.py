#!/usr/bin/env python3

import os
import sys
import glob

def cleanup(path):
    # check if the directory actually contains a dump
    for f in ["AREALIGHTS", "CAMERA", "INFLIGHTS", "MANIFEST", "SAMPLER", "SCENE"]:
        if not os.path.exists(os.path.join(path, f)):
            raise Exception(f'Missing scene file: {f}')

    for t in ["TEX", "STEX", "FTEX", "IMGPART", "MAT"]:
        files = glob.glob(os.path.join(path, f'{t}*'))
        for f in files:
            os.remove(f)    

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} DUMP-DIR")
        sys.exit(1)

    cleanup(sys.argv[1])
