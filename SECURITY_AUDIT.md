# Security audit — Workshop M1 (Yanshee: sensors, dashboard, floor map)

Date: 8 October 2026 · Branch `ilyas` · Author: Ilyas (with Claude Code)

## Scope and method

- **Audited:** the code written for this workshop: `capteurs_serveur.py`,
  `cartographie.py`, `marche_obstacle.py`, `interface.html`, `demo.ps1`, `verifier_api.py`.
- **Method:** code review only. Every input that comes from the network (URL, query
  parameters, request body, responses from other servers) was followed to every
  sensitive action (files, commands, robot movement, safety alerts).
- **Not done, on purpose:** no test, scan or login attempt against the robot. We do
  not own it. Nothing was changed on the robot, and no fix below limits who can reach
  it. The robot-side remarks in section 3 come only from reading our own code, and
  any decision about them belongs to the owner.

## Summary

| # | Finding | Where | Severity | Fix in our code only |
|---|---|---|---|---|
| 1 | Anyone on the Wi-Fi can **hide a real gas or heat alert** with `/simuler` | `capteurs_serveur.py` | **High** | Yes |
| 2 | Anyone on the Wi-Fi can **erase obstacles from the map** (`POST /reference`) | `cartographie.py` | Medium | Yes |
| 3 | No password on the POST routes, and any website is allowed to call them | both servers | Medium | Yes, optional |
| 4 | The top-down camera image can be read by anyone on the Wi-Fi | `cartographie.py` | Low | Yes, optional |

No cross-site scripting, command injection, path traversal, unsafe data loading or
hardcoded secret was found (details in section 4).

---

## 1. Hiding a real alert with `/simuler` — High

**Where:** `capteurs_serveur.py`, `Simulateur.lancer()` (line ~309) and `appliquer()` (line ~356).

**Problem:** the simulation *replaces* the real gas or temperature reading with a
value that moves toward `pic`. Nothing checks `pic` or the durations, and the route
has no password. So `pic` can be set **below** the real value, for as long as you like.

**Scenario:** someone on the same Wi-Fi sends

```
POST http://<server>:8080/simuler?type=gaz&pic=0&palier=100000
```

For about 28 hours the server reports gas = 0. A real leak no longer triggers an
alert, and the robot does not stop and does not warn anyone. The same works for
temperature (`type=temperature&pic=20`). The dashboard does show "SIMULATION"
in the Simulation card, but the alert banner and the robot's reaction are gone.

**Fix (our code only, behaviour unchanged for the dashboard buttons):**

- A simulation may only **raise** the value: `simulated value = max(real, simulated)`.
- Clamp `pic` to the sensor's range (gas ≤ 1023, temperature ≤ 50).
- Cap each duration (`montee`, `palier`, `descente`), for example at 60 s.

```python
# in Simulateur.appliquer()
simule = int(round(base + (s["pic"] - base) * self._facteur(e)))
brut[s["cle"]] = max(reel, simule) if reel is not None else simule
```

---

## 2. Erasing obstacles from the map — Medium

**Where:** `cartographie.py`, `POST /reference` → `prendre_reference()` (line ~226);
relayed by `capteurs_serveur.py` (`do_POST`, `/reference`).

**Problem:** the "empty floor" photo can be retaken by anyone, at any time. The
only check is that the robot (marker 10) is not visible.

**Scenario:** while obstacles are in the zone, someone sends `POST /reference`.
The obstacles become part of the "empty floor" and disappear from the map. With
`--carte`, the robot no longer avoids them and relies only on its front ultrasound
sensor (one narrow beam).

**Fix (our code only):**

- Require a shared token for `/reference` (see finding 3).
- Show the time of the last empty-floor photo in the dashboard, so a change can be
  seen. Optionally, `marche_obstacle.py` can refuse to start if the photo changed
  less than N seconds ago.

---

## 3. No password on POST routes, and every website allowed — Medium

**Where:** `capteurs_serveur.py` and `cartographie.py`, `_envoyer()` / `_repondre()`
(header `Access-Control-Allow-Origin: *`).

**Problem:**

- `/simuler` and `/reference` change how the robot behaves, but anyone who can reach
  the server can call them.
- A POST with no body is a "simple" request, so a web page opened on any computer on
  the network can send it as well, without a CORS preflight.

**Fix (optional, our code only, does not limit access to the robot):**

- Add an optional `JETON` environment variable. If it is set, POST routes require
  the header `X-Jeton: <value>`. If it is not set, nothing changes.
- Keep `Access-Control-Allow-Origin: *` on GET routes: the dashboard opened as a
  local file (`file://`) needs it.

---

## 4. Camera image readable by anyone on the network — Low

**Where:** `cartographie.py`, `GET /carte.jpg`, server listening on `0.0.0.0:8081`.

**Problem:** the straightened top-down view of the zone can be read without a
password by any device on the Wi-Fi. Because every website is allowed, a web page
can read it too. It only shows the floor zone, but people walking in it are visible.

**Fix (optional):**

- Add a `HOTE_CARTE` variable (default `0.0.0.0`). When everything runs on the PC,
  setting it to `127.0.0.1` keeps the image on that PC only.
- Leave the default unchanged, because when `capteurs_serveur.py` runs on the robot,
  the robot has to reach port 8081.

---

## Section 3 — Robot-side remarks (for the owner, not tested, nothing changed)

These are not in our code. We noticed them because our scripts depend on them. We
did **not** verify them on the robot, and any decision is up to the owner.

- **Robot API on port 9090 (YanAPI):** our scripts make the robot walk, turn and
  speak through this API without any login. Anyone on the same Wi-Fi can probably do
  the same.
- **SSH as `pi` with a password:** that is how we copy files to the robot. If the
  default Raspberry Pi password is still in use, anyone on the Wi-Fi could log in.
- **Shared network:** the actual risk of every point in this audit depends on who
  else is on that Wi-Fi.

Possible options for the owner, only if they wish: a dedicated Wi-Fi network for the
robot, changing the `pi` password, a firewall rule on the robot.

---

## Section 4 — Checked and cleared

- **Cross-site scripting (`interface.html`):** all server data is written as plain
  text (`textContent`). The only `innerHTML` clears a list. The SVG map only
  receives numbers.
- **Server-side request forgery (relay of `/carte`):** the target server comes from
  the `CAMERA_HTTP` setting. A request can only change the path, not the host.
- **Command injection and path traversal:** request paths are never used as file
  names or in commands. `cartographie.py` only writes `sol_reference.png`. `stty`
  gets the serial port from a setting or a fixed list. `demo.ps1` only uses its own
  command-line options.
- **Unsafe data loading:** responses are only read as JSON. The numbers sent to
  `/simuler` are converted with `int` or `float`.
- **Secrets:** none in the code.

## Recommended order

1. Finding 1 (`/simuler` may only raise values, with capped durations). Small change, removes the main safety risk.
2. Finding 3 (optional token): it also protects finding 2.
3. Findings 2 and 4 as needed.

None of these fixes touch the robot or limit its access. They only change the
programs written for the workshop.
