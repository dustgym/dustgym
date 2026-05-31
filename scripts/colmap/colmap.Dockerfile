# foss_ipex COLMAP lane (M2b) — offline SfM/MVS "best-achievable reconstruction"
# benchmark that complements the online rtabmap SLAM lane.
#
# Base: graffitytech/colmap 3.12.2 built against CUDA 12.8.1 / Ubuntu 24.04. This
# tag is DRIVER-MATCHED to the host RTX 4090 (CUDA 12.8 driver, 570.x), so the
# CUDA feature extractor / patch-match stereo run on the GPU with a plain
# `--gpus all` and NO `NVIDIA_DISABLE_REQUIRE` work-around (the usual escape hatch
# when an image's CUDA runtime is NEWER than the host driver — not needed here).
#
# We add only python3 + numpy: colmap_recon.py is a thin orchestrator (subprocess
# the `colmap` CLI) plus a pure-numpy Umeyama/Sim3 alignment of the recovered
# camera centers to the Godot ground-truth poses. eval_schema.py is mounted in at
# run time (frozen L0 seam, read-only) — it is stdlib-only, so nothing else is
# needed to emit the TrajectorySample JSON.
#
# CC0-1.0 (see /LICENSE in the repo). The base image carries COLMAP's own
# (BSD-3-Clause) + CUDA EULA terms; see scripts/colmap/README.md.
FROM graffitytech/colmap:3.12.2-cuda12.8.1-devel-ubuntu24.04

# Non-interactive apt; the base is devel-ubuntu24.04 (python3 may be absent/minimal).
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3 \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# The base image's ENTRYPOINT may be `colmap`; reset it so we can drive python3
# (or an interactive shell) without fighting the entrypoint. colmap_recon.py
# itself invokes the `colmap` binary by absolute name found on PATH.
ENTRYPOINT []

WORKDIR /work

# Sanity-surfacing default: print the COLMAP + python versions. The real run is
#   docker run --rm --gpus all -v <repo>:/work fossipex/colmap:m2b \
#       python3 scripts/colmap/colmap_recon.py recon ...
CMD ["bash", "-lc", "colmap --help | head -n 1 && python3 --version && python3 -c 'import numpy; print(\"numpy\", numpy.__version__)'"]
