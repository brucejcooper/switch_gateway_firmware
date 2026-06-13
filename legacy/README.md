# legacy/

Reconstructed/archived tooling for **superseded** firmware. Nothing here is
built by CMake or used by the current C firmware — keep it out of the build.

## `gen_bindings_txt.py`

Generates a Lua-era `/bindings.txt` from `Switch wiring.xlsx`.

Historically a `bindings.txt` was produced from the spreadsheet and **manually
copied** onto the device's littlefs, where the old Lua firmware (`src/main.lua`,
since removed) read it from `/bindings.txt` at boot. No committed tool ever
generated it — the spreadsheet→`bindings.txt` step was manual. This script
recreates that step by reusing the (also removed) `bridge/sync_bindings.py`
spreadsheet parsing verbatim and emitting the positional `bindings.txt` format
instead of `replace_mappings` JSON.

Format: one line per `fixture*8 + button` (0..191); each line a comma-separated
list of `device*256 + address` (device = Modbus slave, address = DALI 0..63);
blank line = unbound button.

```sh
# from controller/ :
.venv/bin/python legacy/gen_bindings_txt.py            # writes ./bindings.txt
.venv/bin/python legacy/gen_bindings_txt.py --stdout   # print instead
.venv/bin/python legacy/gen_bindings_txt.py path/to/other.xlsx -o out.txt
```

**The current firmware is C and does not read `/bindings.txt`** — this targets
the historical Lua firmware only. Use it for reference or a one-off regeneration.
