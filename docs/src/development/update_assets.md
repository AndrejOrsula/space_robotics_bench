# Update Assets

Use `./assets/scripts/update.bash` to update static content. The script manages
two git-backed repositories:

- `assets/srb_assets` for asset payloads.
- `docs/src/_graphics` for docs/task thumbnails and visuals.

Update `assets/srb_assets` by default:

```bash
./assets/scripts/update.bash
```

## Bootstrap from Hugging Face

If submodules are unavailable, initialize a repository from a Hugging Face
mirror. For a private mirror, set `HF_TOKEN` or run `hf auth login`.

```bash
./assets/scripts/update.bash --source hf --repo-id OWNER/REPO
```

Set the repository type, revision, and destination with these options. The
destination must be inside the repository:

```bash
./assets/scripts/update.bash --source hf --repo-id OWNER/REPO --repo-type dataset --revision main --path assets/srb_assets
```

To initialize the graphics repository instead, set `--path` to
`docs/src/_graphics`:

```bash
./assets/scripts/update.bash --source hf --repo-id OWNER/REPO --repo-type dataset --revision main --path docs/src/_graphics
```

`--repo-type` defaults to `dataset`. Supported values: `model`, `dataset`, and
`space`.

For a private mirror, set these environment variables, then run the script:

```bash
export SRB_ASSETS_SOURCE=hf
export SRB_ASSETS_HF_REPO_ID=OWNER/PRIVATE_ASSETS_REPO
export SRB_ASSETS_HF_REPO_TYPE=dataset   # or model / space
export SRB_ASSETS_HF_REPO_REVISION=main         # optional

./assets/scripts/update.bash
```

For non-strict builds, `./.docker/build.bash` can bootstrap a target directory
when it is missing or contains only `.git`. Set the matching variables:

- `SRB_ASSETS_...` for `assets/srb_assets`
- `SRB_GRAPHICS_...` for `docs/src/_graphics`

Strict source-provenance builds require populated asset directories. They skip
this bootstrap step.
