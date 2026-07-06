#!/usr/bin/env python3
"""
Read data/1.hdf5 and dump a full schema + sample values to schema.json.
Run with: conda run -n collection python3 read_hdf5_schema.py
"""

import json
import h5py
import numpy as np


def describe(obj):
    if isinstance(obj, h5py.Dataset):
        info = {
            "type": "dataset",
            "shape": list(obj.shape),
            "dtype": str(obj.dtype),
        }
        # sample first element
        if obj.size > 0:
            val = obj[0]
            if isinstance(val, (bytes, np.bytes_)):
                # variable-length bytes — record hex prefix and length
                b = bytes(val)
                info["sample_bytes_len"] = len(b)
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
    path = "data/1.hdf5"
    with h5py.File(path, "r") as f:
        schema = {k: describe(f[k]) for k in f}

    out_path = "data/hdf5_schema.json"
    with open(out_path, "w") as fp:
        json.dump(schema, fp, indent=2)

    print(f"Schema written to {out_path}")


if __name__ == "__main__":
    main()
