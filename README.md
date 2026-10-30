# Generic IOC ioc-supplies

The power-supply family's image, for GEECS (geecs-epics decision 14: setpoint
devices are plain records; decision 11: one image per family). It is built
on epics-containers' `ioc-streamdevice` developer image (2.8.26ec6), so it
carries everything that image does: asyn, StreamDevice, calc, sequencer,
sscan, busy, autosave, std, iocStats and pvlogging. On top of that it adds:

| Module | Version | Source | Reuse (decision 10) |
|---|---|---|---|
| emqComposite | local | ours (`emqComposite/`): the EMQ triplet's polarity sequence, an SNL program | written: nothing to reuse |

## Why ours

geecs-epics' plain supplies (eztest1, fpstest1, tdktest1, emqtest1-ps) run
on the upstream `ioc-streamdevice` runtime, with their protocols mounted from
the instance. The EMQ triplet's composite cannot. The maintainer ruled
(2026-10-30, geecs-epics #82) that its zero-current polarity sequence is EPICS
records plus an SNL state machine.

The upstream runtime carries `libseq` and `snc` but no compiler. An SNL
program has to be translated, compiled and linked into the IOC, with its
registrar in the dbd, so it cannot be added to a published runtime. The
program therefore lives here.

The image is named by family (decision 11: "by family when more drivers will
join it"). It carries the same modules at the same versions as the upstream
image, so the family's StreamDevice supplies can move onto it when that
helps.

## Build

No vendor code is inside, so the image is published. CI builds on every
push. On a git tag (CalVer `YYYY.M.N`, never re-pushed) it pushes
`ghcr.io/geecs-bella/ioc-supplies-runtime:<tag>` and
`ghcr.io/geecs-bella/ioc-supplies-developer:<tag>`. A pre-release tag
(`2026.10.1b4`) publishes a branch for bench work.

CI runs the template's test, and then `tests/run-emq-test.sh`. That test
starts the IOC with the composite's records, local stand-ins for the unit
and the relay, and one `emqComposite` instance. It checks that a flip ends
"polarity neg" and that an out-of-range put is refused.

The repository is public. The ghcr package was created while the repository
was private, and it keeps its own visibility until that is changed in the
organisation's package settings. Until then a host cannot pull anonymously,
and bench proofs load a build of the tagged commit with `docker load`.

For development only, a local build:

    docker build --platform linux/amd64 --target runtime -t ioc-supplies:local .

ibek-support is pinned to `8545fa2`, the commit the base image was built
with.

The image creates the user `emq_composite` (uid/gid 1990). The EMQ
composite's container runs as it, so that its CA links write under a name
no other container has. The image's default user stays root.

`ibek runtime generate2` removes and recreates `/epics/runtime`, so that user
owns `/epics` itself (not recursively) and `/epics/runtime`. Inside its own
container it could therefore rename entries of `/epics`. The container holds
nothing else of value, and the composite's config is mounted read-only.

## emqComposite

The program `emqComposite` runs once per magnet, after iocInit:

    seq emqComposite, "P=undulator:emqtest1:,CH=ch1,SUP=undulator:emqtest1_ps:,PLC=undulator:plctest1:,COIL=15"

It sequences the composite's records, which belong to the instance
(geecs-epics `services/undulator/supplies/emqtest1/config/`): the signed
setpoint, the output request, `busy`, `state`, `message`, `reset` and the
named values.

**Trusted readbacks.** The unit's current, its output and the relay's coil
count only while they are connected and below MAJOR severity. A decision or
a wait that meets an untrusted one aborts.

**A setpoint put** is refused by the program if `|value|` exceeds `imax`
(the record has no output link, so its alarm alone stops nothing).
- **Same polarity, or 0 A:** the program sets the unit's current directly,
  but only while the unit's output is on. An output that is off stays off
  until the operator turns it on with `output_CH:SP`, the explicit intent.
- **The other sign:** the program runs the polarity sequence.
  1. Set the unit to 0 A.
  2. Wait for |I| below the threshold, held for the hold time.
  3. Turn the output off, and wait for it to read off. This step is skipped
     if the output was off already.
  4. With the output off, wait for |I| below the threshold, held again.
  5. Write the coil (`{PLC}do_ch{COIL}:SP`), and wait for its readback.
  6. Wait the contact settle time. The coil bit is the PLC's register, not
     the contact.
  7. Turn the output back on, and wait for it to read on. **Only if it was
     on before the sequence.**
  8. Set |value|, and wait until the unit's programmed current reads it.

**Aborts.** Any failed write, timeout, mismatch or untrusted readback turns
the output off. The relay is never written again. The channel then latches:
- `state` goes to `aborted` (MAJOR), and `busy` to 1;
- `message` says why;
- the output is read back. Until it reads off, the program retries the
  off every 2 s and says "OFF NOT CONFIRMED".

An abort while current still flows (for example "I did not fall") turns the
output off under load. That is the specified behaviour, and it is the
inductive-kick case in the maintainer's open flyback question.

**Reset.** A reset is taken only in `aborted`, only if it was made after the
abort (its event flag is cleared on abort entry), and only once the output
reads off. A reset that arrives in any other state is consumed and does
nothing. The instance's access security refuses it anyway unless the state
is `aborted`.

**Relay watch.** While the program is idle, and before the coil write, a coil
readback that changes or stops being trusted aborts with the output off.
That catches a relay moved by anyone else: a PLC reboot, NI's
shared-variable engine, a spoofing client. It cannot prevent the hot switch,
only limit it.

**Puts during a sequence.** `busy` goes to 1 as the put is taken. The
instance's access security refuses setpoint and output puts while `busy` is
1. A put that slips into the instant before is dropped when the sequence
ends, not queued.

**At start**, every channel is latched ("restarted: reset to resume") and
nothing is written. A restart mid-sequence resumes nothing:
- after step 3, the output stays off;
- during steps 1-2, the unit is left at 0 A with its output on;
- the relay has not moved unless step 5 completed.

**Timeouts** are measured with the clock, not `delay()`, because `delay()`
restarts on a self-transition. The waiting states poll every 0.1 s.

**Three instances, one chain.** The instance runs one program per magnet, and
nothing serialises them. Three can sequence at once on the units' shared
9600-baud chain. Each write waits at most the protocols' lock timeout (5 s)
for the line, and the composite's output timeout (15 s) is above it.

**Links.** Every PV is a CA channel. `option -c` starts the program without
waiting for them to connect. The writes go out under the IOC's process user,
`emq_composite`, from the loopback, which is what emqtest1-ps
(`EMQ_PS_WRITER`, ASG `SUPPLY_LOCAL`) and plctest1 (`PLC1_EMQ_COMPOSITE`,
ASG `EMQ_COMPOSITE`, both with a loopback HAG under `asCheckClientIP`)
grant.

## Licence

GEECS's own work here is under `LICENSE`, the GEECS licence: BSD-style,
Copyright The Regents of the University of California, through Lawrence
Berkeley National Laboratory, as in GEECS-Plugins. That covers the
Dockerfile changes, `emqComposite`, the ibek support file under
`ibek-support-geecs/` and the docs. The files generated from epics-containers'
ioc-template keep their Apache-2.0 licence (`LICENSE-ioc-template`). The EPICS
modules come with the base image (not vendored) and carry their own
licences.

## Template Info

This repository was generated by
[ioc-template](https://github.com/epics-containers/ioc-template) 4.8.0. The
first commit is the generator's files, untouched. Its `.gitmodules` and the
ibek-support submodule, which copier's task would add, arrive with the next
commit, pinned
(`copier update -a --trust .` to update). The devcontainer and `./build`
work as in every epics-containers generic IOC.
