# Event-Camera Weight Mirrors

Fetch checkpoints with `srb event-camera fetch-weights`. It verifies SHA256 and
aborts on mismatch. **SRB does not redistribute weights in the repository.**

| Artifact | `--artifact` value | Filename | Consumer |
| --- | --- | --- | --- |
| E2VID reconstruction | `e2vid` (default) | `E2VID_lightweight.pth.tar` (~41 MiB) | `srb.datasets.event_validation.reconstruction.e2vid_reconstruct` |
| V2CE learned simulator | `v2ce` | `v2ce_3d.pt` | `srb.core.sensor.event_camera.backends.v2ce.V2CEBackend` |

E2VID's checkpoint comes from `rpg_e2vid`. SRB's runtime adapter uses the MIT
`e2vid` package from `uzh-rpg/e2calib` via `srb[event-reconstruction]`; do not
vendor GPL model code into SRB.

Both maintainer-pinned mirrors are in the public Hugging Face repository
`AndrejOrsula/srb-event-camera-weights`.

| Mirror | Artifact | SHA256 |
| --- | --- | --- |
| `hf-andrejorsula-e2vid` | `E2VID_lightweight.pth.tar` | `4cfeb2c850bf48fc9fa907e969cb8a04e3c51314da2d65bdb81145ac96574128` |
| `hf-andrejorsula-v2ce` | `v2ce_3d.pt` | `97920e0c3f3c1df55f12800c166bf1de11466f7546bdb7b40261c5a14cddef26` |

Upstream sources:

- E2VID: `http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar`
- V2CE: `https://drive.google.com/file/d/1-aC6CTGZgAZk3snANZ46FAGNkPzu_Scw`

## Fetching weights

Fetch a registered mirror or provide a URL and digest:

```sh
srb event-camera list-mirrors
srb event-camera fetch-weights --mirror hf-andrejorsula-e2vid
srb event-camera fetch-weights --mirror hf-andrejorsula-v2ce

srb event-camera fetch-weights --artifact e2vid \
  --url https://example.com/path/to/E2VID_lightweight.pth.tar \
  --sha256 <hex digest>
```

Mirrors pin artifact identity and verified SHA256; `--mirror` ignores
`--artifact` and `--sha256`. Files go to `~/.cache/srb/event_weights/`, or
`$XDG_CACHE_HOME/srb/event_weights/` when set. Private Hugging Face URLs accept bearer tokens
from `HF_TOKEN`, `HUGGING_FACE_HUB_TOKEN`, or the Hugging Face CLI token cache.

## Adding a mirror

Add a `WeightMirror` to `KNOWN_MIRRORS` in
`srb/core/sensor/event_camera/weights.py`:

```python
WeightMirror(
    short_name="huggingface-<owner>-e2vid",  # include the artifact kind
    artifact=WeightArtifact(
        name="E2VID_lightweight.pth.tar",
        url="https://huggingface.co/<owner>/<repo>/resolve/main/E2VID_lightweight.pth.tar",
        sha256="<hex digest>",
        size_bytes=<file size>,
    ),
    license_note="GPL-3.0, © 2019 Rebecq et al.",
    notes="Mirrored 2026-MM-DD from rpg_e2vid release X.Y.Z.",
),
```

Each mirror must:

1. Fetch through `urllib.request` without consent interstitials or
   rate-limited redirects; verify with
   `srb event-camera fetch-weights --url <url> --sha256 <sha> --force`.
1. Serve the exact upstream checkpoint. Give distinct `short_name` and
   `artifact.name` values for modified or retrained checkpoints.
1. Use a stable URL, not ephemeral or signed storage.
1. Pin the maintainer-verified SHA256 of a known-good download.
1. Record the upstream license in `license_note`: `rpg_e2vid` published the
   E2VID checkpoint under GPL-3.0; SRB's E2VID Python provider
   (`uzh-rpg/e2calib`) and V2CE-Toolbox are MIT.
