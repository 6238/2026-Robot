# Style guide

House rules for Team 6238 / `2026-Robot`.

## Branch naming

All branches must use:

```text
<firstname>-<last-initial>/<dash-delimited-short-name>
```

- `firstname`: lowercase given name of the author
- `last-initial`: lowercase first letter of the author's last name
- short name: lowercase, dash-delimited, concise (feature/fix topic)

**Good**

```text
maxim-g/spotless-pre-push
maxim-g/hopper-jam-fix
liav-a/vision-std-devs
```

**Bad**

```text
maxim-experimental
fix/update-stale-unit-tests
feature/MyFeature
MAXIM-G/Foo
```

If the author's name is unknown, ask before creating a branch.

Also enforced in `.cursor/rules/branch-naming.mdc`.

## Formatting (Spotless)

- Formatter: **Google Java Format** via Spotless
- Spotless runs automatically before every `compileJava`
- Manual apply: `./gradlew spotlessApply` (Windows: `.\gradlew.bat spotlessApply`)
- Covered files: `.java`, `.gradle`, `.json`, `.md`, `.gitignore`
- Do not fight Spotless with a different IDE formatter

### Pre-push check

Pushes run Spotless via `.githooks/pre-push` (`spotlessCheck`). Enable once per clone:

```bash
git config core.hooksPath .githooks
```

If the check fails, run `./gradlew spotlessApply`, commit the formatting, then push again.

## Generated / build artifacts

Do not commit build or test outputs. These are gitignored (see `.gitignore`), including:

- `build/`, `.gradle/`, `bin/`, `out/`
- `*.wpilog`, `*.hlog`, `logs/`
- `src/main/java/frc/robot/BuildConstants.java` (regenerated every build)

**Exception:** committed robot config under `src/main/java/frc/robot/generated/` (e.g. TunerConstants) is intentional and stays tracked.

## Related docs

- [`AGENT.md`](AGENT.md) — agent commands and architecture
- [`AGENT_SETUP.md`](AGENT_SETUP.md) — cold-start environment setup
