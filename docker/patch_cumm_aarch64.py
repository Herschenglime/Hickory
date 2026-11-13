"""Port cumm/spconv sources to aarch64 + CUDA 13 (used by Dockerfile.aarch64).

usage: python patch_cumm_aarch64.py <cumm_root> <spconv_root>

Each edit fixes a concrete failure seen when building on a GB10 (aarch64, sm_121, CUDA 13):
  1. cumm only looks for conda CUDA headers in targets/x86_64-linux and libs in lib64;
     conda on aarch64 uses targets/sbsa-linux and lib.
  2. CUDA 13 (CCCL 3 / libcu++) requires C++17; cumm/spconv default to C++14.
  3. sm_121 is missing from cumm's whitelist of CUDA architectures.
"""
import sys
from pathlib import Path

cumm, spconv = Path(sys.argv[1]), Path(sys.argv[2])


def edit(path, old, new, count=1):
    t = path.read_text()
    assert t.count(old) == count, f"{path}: expected {count} x {old!r}, found {t.count(old)}"
    path.write_text(t.replace(old, new))


edit(
    cumm / "cumm/common.py",
    '''                lib = Path(nvcc_path).parent.parent / "lib64"
                include = Path(nvcc_path).parent.parent / "targets/x86_64-linux/include"
''',
    '''                _cuda_root = Path(nvcc_path).parent.parent
                lib = _cuda_root / "lib64"
                if not lib.exists():
                    lib = _cuda_root / "lib"
                include = _cuda_root / "targets/x86_64-linux/include"
                if not include.exists():
                    # conda CUDA on other platforms, e.g. targets/sbsa-linux (aarch64)
                    include = next((d for d in sorted((_cuda_root / "targets").glob("*-linux/include"))
                                    if (d / "cuda.h").exists()), include)
''',
)
edit(cumm / "cumm/common.py", '"10.0", "12.0",', '"10.0", "12.0", "12.1",')
for f, n in [(cumm / "cumm/__init__.py", 1), (cumm / "setup.py", 1), (cumm / "cumm/nvrtc/__init__.py", 4), (spconv / "setup.py", 1)]:
    edit(f, '"c++14"', '"c++17"', count=n)
print("cumm/spconv patched for aarch64 + CUDA 13")
