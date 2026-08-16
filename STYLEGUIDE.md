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

## Pull requests

Titles: short and operative (what changed).

Bodies: lean operative paragraphs — `Add <thing> to <do X> because <reason>.` Explain briefly why this solves the problem and why it does not break existing behavior.

Do **not** include `Made with Cursor` or other tool footers. Prefer `gh pr create --body-file …` so nothing appends extras.

Also enforced in `.cursor/rules/pull-request-style.mdc`.

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

## Code

- Java **17**; match existing package layout under `frc.robot`
- Prefer the existing AdvantageKit pattern: `FooIO` + hardware/sim impls + subsystem that owns `FooIOInputsAutoLogged` — details in [`AGENTS.md`](AGENTS.md)
- Keep diffs focused; don’t reformat or refactor unrelated code

## Tests

- Mock the `IO` interface, not the subsystem class
- In `@BeforeEach`, set all `*Connected` flags to `true` in `updateInputs` (avoids alert/rumble NPEs)
- Use `command.initialize()` for simple `runOnce`/`runEnd` commands; add `HAL.initialize(500, 0)` in `@BeforeAll` when testing timing/`Alert`

## Related docs

- [`INSTALLATION_INSTRUCTIONS.md`](INSTALLATION_INSTRUCTIONS.md) — install toolchain, build, deploy
- [`AGENTS.md`](AGENTS.md) — agent commands and architecture
- [`AGENT_SETUP.md`](AGENT_SETUP.md) — cold-start environment setup
