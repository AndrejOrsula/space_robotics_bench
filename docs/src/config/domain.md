# Environment Configuration — Domain

Set `env.domain` to choose a planetary or orbital domain. Each domain provides
default gravity and lighting. Scenery provides terrain. Domains have no
atmospheric model.

## Available Domains

Choose one of five domains. No shipped task defaults to `earth` or `asteroid`:

| Domain | Description |
| ------------------ | --------------------------------------- |
| `moon` [default] | Lunar surface |
| `mars` | Martian surface |
| `earth` | Earth surface |
| `asteroid` | Low-gravity body |
| `orbit` | Orbital microgravity environment |

Set the domain on the command line:

```bash
# Run a simulation on Mars
srb agent teleop --env sample_collection env.domain=mars
```

Gravity affects free bodies and mobile bases. Shipped manipulators disable
gravity. To use gravity from another domain, set `env.gravity=<domain>`. This
also uses that domain's gravity randomization. `modular_assembly` defaults to
`earth` gravity.

Lighting sets intensity, color temperature, angular diameter, and skydome. To
set fog, use `env.visuals.fog`. Rendered irradiance is not calibrated
atmospheric lighting.

Variant resolution excludes candidates whose nonempty `DOMAINS` omits the selected domain; an empty list permits all. This rule applies only to variants.
