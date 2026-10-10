# Event-Camera Fixture Mirrors

Use the registered Prophesee Gen4-HD active-marker recording in frozen
`EVENT_HDF5` schema v1, the **Gen4 reference fixture**, to test the schema,
importer, statistics, and opt-in E2VID path. It is not calibration
data.

**SRB ships no captures in git.** Ownership and publication rights for each
recording depend on its source and the terms that govern that recording. Check
those terms before redistributing a capture or publishing a mirror.

## Fetching fixtures

List and fetch a registered mirror or provide a URL and digest:

```sh
srb event-camera list-fixtures
srb event-camera fetch-fixture --mirror hf-andrejorsula-active-marker-gen4

srb event-camera fetch-fixture \
  --url https://your-host.example/gen4_reference.h5 \
  --sha256 <hex digest>
```

Mirrors pin schema version and maintainer-verified SHA256. Downloads go to
`~/.cache/srb/event_fixtures/` by default, or `$XDG_CACHE_HOME/srb/event_fixtures/` when `XDG_CACHE_HOME` is set.

## Prophesee Active Marker mirror

The `hf-andrejorsula-active-marker-gen4` mirror is in the public
`AndrejOrsula/srb-event-camera-fixtures` dataset. Prophesee's terms govern
redistribution.

To rebuild it, convert Prophesee's `active_marker` share
(https://kdrive.infomaniak.com/app/share/975517/d39fd010-8d56-4ce1-980d-224d5654b478;
ZIP SHA256 `3a2aced3efee1b37b89e53bcb8b2882985ff6da9b556e7cfa1494e42e509098f`),
extract `MV_SDK_5_x/active_marker_dataset/active_marker.raw`, then import it
with `srb dataset import-raw-evt3` (see
[RAW EVT3 import](event_camera.md#raw-evt3-import)).

The import contains 22,316,758 events at 1280x720.

## Adding a fixture mirror

Add a `FixtureMirror` to `KNOWN_FIXTURES` in
`srb/core/sensor/event_camera/fixtures.py`:

```python
FixtureMirror(
    short_name="zenodo-<owner>-gen4",  # include the sensor kind
    artifact=FixtureArtifact(
        name="gen4_reference.h5",
        url="https://zenodo.org/records/<id>/files/gen4_reference.h5",
        sha256="<hex digest>",
        size_bytes=<file size>,
        schema_version=1,
        sensor_model="prophesee_gen4_hd",
        num_episodes=<count>,
    ),
    license_note="CC-BY-4.0, © 2026 <owner/lab>",
    notes="Captured 2026-MM-DD on EVK4-HD, <scene summary>.",
),
```

Each mirror must:

1. Fetch through `urllib.request` without consent interstitials or expiring
   signed URLs; verify with
   `srb event-camera fetch-fixture --url <url> --sha256 <sha> --force`.
1. Serve [EVENT_HDF5 schema v1](event_camera.md#hdf5-layout) with
   `EventEpisodeMetadata` reproducibility attributes. Give modified or
   resampled captures distinct `short_name` and `artifact.name` values.
1. Pin the sensor model in `FixtureArtifact.sensor_model`.
1. Use stable hosting (Zenodo, Hugging Face datasets, or an institutional
   research-data repository), not ephemeral or presigned storage.
1. Pin the maintainer-verified SHA256 of a known-good download.
1. Record capture licensing in `license_note` (often `CC-BY-4.0` for academic
   captures); never exceed upstream terms.
