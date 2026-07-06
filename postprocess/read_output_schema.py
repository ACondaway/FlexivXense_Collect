#!/usr/bin/env python3
"""
Read a converted HDF5 episode file and dump its schema to JSON.

Usage:
    conda run -n collection python3 postprocess/read_output_schema.py \
        --hdf5 /path/to/output.hdf5 \
        [--out  /path/to/schema.json]   # default: same dir as hdf5

Output JSON format matches data/hdf5_schema.json.
"""

import argparse
import json
import os

import h5py
import numpy as np


def describe(obj):
    if isinstance(obj, h5py.Dataset):
        info = {
            "type": "dataset",
            "shape": list(obj.shape),
            "dtype": str(obj.dtype),
        }
        if obj.size > 0:
            val = obj[0]
            if isinstance(val, (bytes, np.bytes_)):
                b = bytes(val)
                info["sample_bytes_len"] = len(b.rstrip(b"\x00"))
                info["sample_hex_prefix"] = b[:8].hex()
            elif isinstance(val, np.ndarray):
                info["sample"] = val.flatten()[:8].tolist()
            else:
                info["sample"] = val.tolist() if hasattr(val, "tolist") else str(val)
        return info
    elif isinstance(obj, h5py.Group):
        return {"type": "group", "children": {k: describe(obj[k]) for k in obj}}
    return {"type": "unknown"}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hdf5", required=True, help="Path to converted HDF5 file")
    p.add_argument("--out",  default=None,  help="Output JSON path (default: alongside HDF5)")
    args = p.parse_args()

    out_path = args.out or os.path.splitext(args.hdf5)[0] + "_schema.json"

    with h5py.File(args.hdf5, "r") as f:
        schema = {k: describe(f[k]) for k in f}

    with open(out_path, "w") as fp:
        json.dump(schema, fp, indent=2)

    print(f"Schema written to {out_path}")


if __name__ == "__main__":
    main()
