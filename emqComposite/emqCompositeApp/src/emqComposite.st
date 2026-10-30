/* emqComposite: the polarity sequence of one bipolar EMQ magnet (U_EMQTripletBipolar).
 *
 * One instance per magnet, after iocInit:
 *   seq emqComposite, "P=undulator:emqtest1:,CH=ch1,SUP=undulator:emqtest1_ps:,PLC=undulator:plctest1:,COIL=15"
 * The composite's records (geecs-epics services/undulator/supplies/emqtest1/config/
 * emq_composite.template) hold the signed setpoint, the output request, busy, state, message,
 * reset and the named values. This program only sequences. Access security refuses the
 * operator's setpoint and output puts while busy_CH is 1, and reset_CH unless state_CH is
 * "aborted".
 *
 * Readbacks. i_meas (the unit's measured current), out_rbv (its output) and coil_rbv (the relay
 * coil) are trusted only while connected and below MAJOR severity. Mid-sequence, any decision
 * or wait that meets an untrusted one aborts. Immediately before the coil write, the current
 * and the output are read again (pvGet, 2 s) and must be trusted and stamped within the last
 * 2.5 s (the unit polls them every 1 s): a readback that stopped updating without an alarm
 * cannot pass.
 *
 * A setpoint put (|value| <= imax, else refused here: the record has no OUT link):
 *   - with the relay's present polarity, or 0 A: the unit's current is set directly, only while
 *     its output is on, and the programmed current is read back;
 *   - with the other sign: the polarity sequence
 *       0 A -> |I| < threshold held (zero timeout)
 *           -> output off, read back (output timeout); skipped if it was off already
 *           -> |I| < threshold held again with the output off (zero timeout)
 *           -> a fresh read of current and output -> coil write, coil readback (coil timeout)
 *           -> contact settle time
 *           -> output on, read back: only if it was on before the sequence, and only if the
 *              unit shows no fault, front-panel off or interlock
 *           -> |value|, read back as the unit's programmed current.
 *     If the output goes off during the sequence without the program (front panel, interlock),
 *     the sequence aborts: it never turns back on an output someone else turned off.
 * An output put (output_CH:SP, the operator's explicit intent) switches the unit's output and
 * reads it back. Nothing else turns an output on.
 *
 * Abort: any failed write, timeout, mismatch or untrusted readback mid-sequence. The output is
 * turned off; until it reads off (trusted) the program retries every 2 s. The relay is never
 * written again. state "aborted" (MAJOR), busy 1, latched until a reset put made AFTER the
 * abort and only once the output's state is known (off, after an abort).
 *
 * Relay watch while idle: a coil readback that CHANGES aborts (output off): the relay moved. A
 * coil readback that is merely untrusted (a failed PLC poll, a plctest1 restart) does not touch
 * the output: the magnet goes to "relay_unknown" (MAJOR, puts refused) and back to idle when
 * the readback is trusted again and unchanged; changed, it aborts.
 *
 * At start every channel is latched ("restarted"), nothing is written, and a reset is accepted
 * once the output's readback is trusted. Timeouts use the clock: delay() restarts on a
 * self-transition.
 */
program emqComposite("P=,CH=,SUP=,PLC=,COIL=")

option +r;      /* reentrant: one instance per magnet */
option -c;      /* start without waiting for every channel to connect */

%%#include <math.h>
%%#include <stdio.h>
%%#include <string.h>
%%#include <epicsTime.h>

/* The composite's own records */
double sp;            assign sp           to "{P}current_limit_{CH}:SP";   monitor sp;
evflag sp_ev;         sync sp sp_ev;
short  out_req;       assign out_req      to "{P}output_{CH}:SP";          monitor out_req;
evflag out_ev;        sync out_req out_ev;
short  busy;          assign busy         to "{P}busy_{CH}";
short  step;          assign step         to "{P}state_{CH}";
string msg;           assign msg          to "{P}message_{CH}";
short  reset;         assign reset        to "{P}reset_{CH}";              monitor reset;
evflag reset_ev;      sync reset reset_ev;
short  neg_coil;      assign neg_coil     to "{P}config:negative_when_coil_{CH}"; monitor neg_coil;
double imax;          assign imax         to "{P}config:imax_a";            monitor imax;
double threshold;     assign threshold    to "{P}config:zero_threshold_a";  monitor threshold;
double hold;          assign hold         to "{P}config:zero_hold_s";       monitor hold;
double zero_timeout;  assign zero_timeout to "{P}config:zero_timeout_s";    monitor zero_timeout;
double out_timeout;   assign out_timeout  to "{P}config:output_timeout_s";  monitor out_timeout;
double coil_timeout;  assign coil_timeout to "{P}config:coil_timeout_s";    monitor coil_timeout;
double settle;        assign settle       to "{P}config:contact_settle_s";  monitor settle;

/* The unit (emqtest1-ps) and the relay (plctest1) */
double sp_raw;        assign sp_raw       to "{SUP}current_limit_{CH}:SP";
double sp_rbv;        assign sp_rbv       to "{SUP}current_limit_{CH}";    monitor sp_rbv;
double i_meas;        assign i_meas       to "{SUP}current_{CH}";          monitor i_meas;
short  out_sp;        assign out_sp       to "{SUP}enable_output_{CH}:SP";
short  out_rbv;       assign out_rbv      to "{SUP}enable_output_{CH}";    monitor out_rbv;
short  fault_rbv;     assign fault_rbv    to "{SUP}fault_{CH}";            monitor fault_rbv;
short  panel_rbv;     assign panel_rbv    to "{SUP}panel_off_{CH}";        monitor panel_rbv;
short  ilk_rbv;       assign ilk_rbv      to "{SUP}interlock_{CH}";        monitor ilk_rbv;
short  coil_sp;       assign coil_sp      to "{PLC}do_ch{COIL}:SP";
short  coil_rbv;      assign coil_rbv     to "{PLC}do_ch{COIL}";           monitor coil_rbv;

/* step (state_CH) values, the mbbo's states in order: 0 idle, 1 zeroing, 2 output_off,
   3 checking, 4 switching, 5 output_on, 6 setting, 7 aborted, 8 settling, 9 output_change,
   10 relay_unknown */

double target;        /* the signed value being set */
short  want_coil;
short  last_coil;     /* the coil readback the program last accepted */
short  was_on;        /* the output's state when the sequence began */
short  want_out;
int    ok;            /* the last write's outcome (CA completion) */
int    stale;         /* the fresh read before the coil write failed */
int    off_confirmed; /* the output's state after an abort is known */
int    coil_ok;       /* reset_check: the coil readback, read again, is trusted */
int    retry_off;     /* 1 after an abort: retry the off until it reads off */
double t0;            /* when this step began */
double since;         /* when |I| first read under the threshold in this hold; < 0: not yet */
double age_i, age_o;
char   reason[40];

%{
static int sign_of(short coil, short neg) { return coil == neg ? -1 : 1; }
static double now_s(void)
{
    epicsTimeStamp t;
    epicsTimeGetCurrent(&t);
    return t.secPastEpoch + t.nsec * 1e-9;
}
static double age_of(epicsTimeStamp ts)
{
    epicsTimeStamp t;
    epicsTimeGetCurrent(&t);
    return epicsTimeDiffInSeconds(&t, &ts);
}
}%

ss polarity {
    state start {
        entry {
            busy = 1;  pvPut(busy, SYNC);
            step = 7;  pvPut(step, SYNC);
            strcpy(reason, "restarted");
            strcpy(msg, "restarted: reset to resume"); pvPut(msg, SYNC);
            off_confirmed = 0;      /* known once the output's readback is trusted */
            retry_off = 0;          /* nothing is written at start */
            efClear(reset_ev);
        }
        when () {} state aborted
    }

    state aborted {
        when (!off_confirmed && !retry_off && pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR) {
            off_confirmed = 1;      /* after a start: the output's state is known, whatever it is */
        } state aborted
        when (!off_confirmed && retry_off && out_rbv == 0 && pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR) {
            off_confirmed = 1;
            sprintf(msg, "ABORT %.22s; off", reason); pvPut(msg, SYNC);
        } state aborted
        when (!off_confirmed && retry_off && delay(2.0)) {
            out_sp = 0; pvPut(out_sp, SYNC);
            sprintf(msg, "%.22s; off unconfirmed", reason); pvPut(msg, SYNC);
        } state aborted
        when (reset && !off_confirmed) {
            reset = 0; pvPut(reset, SYNC);
            efClear(reset_ev);
            sprintf(msg, "%.22s; reset: output?", reason); pvPut(msg, SYNC);
        } state aborted
        when (efTestAndClear(reset_ev) && reset) {
            reset = 0; pvPut(reset, SYNC);
        } state reset_check
    }

    /* A reset is taken only with the relay's coil readback trusted now: connected, read again
     * (pvGet, 2 s) and below MAJOR. Otherwise last_coil would be a guess (0 after a start with
     * plctest1 down), and the coil's real value, once it returned, would read as a moved relay
     * and abort under load. The coil readback posts on change only, so its time stamp is not an
     * age: the read itself, and its severity, are the check. */
    state reset_check {
        entry {
            coil_ok = pvConnected(coil_rbv) && pvGet(coil_rbv, SYNC, 2.0) == pvStatOK
                      && pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR;
        }
        when (coil_ok) {
            efClear(sp_ev); efClear(out_ev);
            last_coil = coil_rbv;
            step = 0;  pvPut(step, SYNC);
            busy = 0;  pvPut(busy, SYNC);
            strcpy(msg, "reset"); pvPut(msg, SYNC);
        } state idle
        when () {
            sprintf(msg, "%.22s; reset: relay?", reason); pvPut(msg, SYNC);
        } state aborted
    }

    state idle {
        when (efTestAndClear(reset_ev)) {             /* refused by access security; consumed */
            if (reset) { reset = 0; pvPut(reset, SYNC); }
        } state idle
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR)) {
            busy = 1;  pvPut(busy, SYNC);
            step = 10; pvPut(step, SYNC);
            strcpy(msg, "relay unknown: readback lost"); pvPut(msg, SYNC);
        } state relay_unknown
        when (coil_rbv != last_coil) {
            sprintf(reason, "relay moved: coil %d", coil_rbv);
        } state abort
        when (efTestAndClear(out_ev)) {
            busy = 1;  pvPut(busy, SYNC);
            efClear(sp_ev);
            want_out = out_req ? 1 : 0;
        } state output_change
        when (efTestAndClear(sp_ev)) {
            busy = 1;  pvPut(busy, SYNC);              /* first: the next put is refused */
            efClear(out_ev);
            target = sp;
        } state decide
    }

    /* The coil's readback is untrusted while idle: the output is not touched. */
    state relay_unknown {
        when (efTestAndClear(reset_ev)) {
            if (reset) { reset = 0; pvPut(reset, SYNC); }
        } state relay_unknown
        when (pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR && coil_rbv == last_coil) {
            efClear(sp_ev); efClear(out_ev);
            step = 0;  pvPut(step, SYNC);
            busy = 0;  pvPut(busy, SYNC);
            strcpy(msg, "relay readback back, unchanged"); pvPut(msg, SYNC);
        } state idle
        when (pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR && coil_rbv != last_coil) {
            strcpy(reason, "relay moved, unknown");
        } state abort
    }

    state output_change {
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR)) {
            strcpy(reason, "output readback lost");
        } state abort
        when () {
            step = 9;  pvPut(step, SYNC);
            out_sp = want_out;
            ok = (pvPut(out_sp, SYNC) == pvStatOK);
            t0 = now_s();
        } state output_change_wait
    }

    state output_change_wait {
        when (!ok) { strcpy(reason, "output write failed"); } state abort
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR)) {
            strcpy(reason, "output readback lost");
        } state abort
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR) || coil_rbv != last_coil) {
            strcpy(reason, "relay moved or lost");
        } state abort
        when (out_rbv == want_out) {
            sprintf(msg, "output %s", want_out ? "on" : "off"); pvPut(msg, SYNC);
            efClear(sp_ev); efClear(out_ev);
            step = 0;  pvPut(step, SYNC);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
        when (now_s() - t0 > out_timeout) { strcpy(reason, "output did not follow"); } state abort
        when (delay(0.1)) {} state output_change_wait
    }

    state decide {
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR)) {
            strcpy(reason, "relay readback lost");
        } state abort
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR)) {
            strcpy(reason, "output readback lost");
        } state abort
        when (!(pvConnected(i_meas) && pvSeverity(i_meas) < pvSevrMAJOR)) {
            strcpy(reason, "current readback lost");
        } state abort
        when (!pvConnected(imax) || !pvConnected(threshold) || !pvConnected(hold) || !pvConnected(settle)
              || !pvConnected(zero_timeout) || !pvConnected(out_timeout) || !pvConnected(coil_timeout)) {
            strcpy(msg, "refused: a named value is missing"); pvPut(msg, SYNC);
            efClear(sp_ev); efClear(out_ev);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
        when (!(fabs(target) <= imax)) {
            sprintf(msg, "refused: %.3f A outside +-%.0f", target, imax); pvPut(msg, SYNC);
            efClear(sp_ev); efClear(out_ev);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
        when ((target == 0.0 || (target > 0) == (sign_of(coil_rbv, neg_coil) > 0)) && out_rbv == 0 && target != 0.0) {
            strcpy(msg, "refused: output is off"); pvPut(msg, SYNC);
            efClear(sp_ev); efClear(out_ev);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
        when (target == 0.0 || (target > 0) == (sign_of(coil_rbv, neg_coil) > 0)) {
            sp_raw = fabs(target);
            ok = (pvPut(sp_raw, SYNC) == pvStatOK);
            t0 = now_s();
        } state direct_wait
        when () {
            want_coil = (target < 0) ? neg_coil : 1 - neg_coil;
            was_on = out_rbv;
            step = 1;  pvPut(step, SYNC);
            sp_raw = 0.0;
            ok = (pvPut(sp_raw, SYNC) == pvStatOK);
            t0 = now_s();
            since = -1;
        } state zeroing
    }

    /* A same-sign set: report the unit's own programmed current, not the CA completion. */
    state direct_wait {
        when (!ok) {
            sprintf(msg, "unit refused %.3f A", fabs(target)); pvPut(msg, SYNC);
            efClear(sp_ev); efClear(out_ev);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
        when (fabs(sp_rbv - fabs(target)) < 0.0005 && pvConnected(sp_rbv) && pvSeverity(sp_rbv) < pvSevrMAJOR) {
            sprintf(msg, "set %.3f A", target); pvPut(msg, SYNC);
            efClear(sp_ev); efClear(out_ev);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
        when (now_s() - t0 > out_timeout) {
            sprintf(msg, "unit did not read back %.3f A", fabs(target)); pvPut(msg, SYNC);
            efClear(sp_ev); efClear(out_ev);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
        when (delay(0.1)) {} state direct_wait
    }

    state zeroing {
        when (!ok) { strcpy(reason, "write 0 A failed"); } state abort
        when (!(pvConnected(i_meas) && pvSeverity(i_meas) < pvSevrMAJOR)) {
            strcpy(reason, "current readback lost");
        } state abort
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR)) {
            strcpy(reason, "output readback lost");
        } state abort
        when (was_on && out_rbv == 0) {
            strcpy(reason, "output off by another");
        } state abort
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR) || coil_rbv != last_coil) {
            strcpy(reason, "relay moved or lost");
        } state abort
        when (now_s() - t0 > zero_timeout) {
            sprintf(reason, "I stuck: %.3g A", i_meas);
        } state abort
        when (since >= 0 && fabs(i_meas) < threshold && now_s() - since >= hold) {
            ok = 1;
            if (was_on) {
                step = 2;  pvPut(step, SYNC);
                out_sp = 0;
                ok = (pvPut(out_sp, SYNC) == pvStatOK);
            }
            t0 = now_s();
        } state output_off
        when (delay(0.1)) {
            if (fabs(i_meas) < threshold) {
                if (since < 0) since = now_s();
            } else {
                since = -1;
            }
        } state zeroing
    }

    state output_off {
        when (!ok) { strcpy(reason, "off write failed"); } state abort
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR)) {
            strcpy(reason, "output readback lost");
        } state abort
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR) || coil_rbv != last_coil) {
            strcpy(reason, "relay moved or lost");
        } state abort
        when (out_rbv == 0) {
            step = 3;  pvPut(step, SYNC);
            t0 = now_s();
            since = -1;
        } state checking
        when (now_s() - t0 > out_timeout) { strcpy(reason, "output not read off"); } state abort
        when (delay(0.1)) {} state output_off
    }

    state checking {
        when (!(pvConnected(i_meas) && pvSeverity(i_meas) < pvSevrMAJOR)) {
            strcpy(reason, "current readback lost");
        } state abort
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR) || out_rbv != 0) {
            strcpy(reason, "output not off");
        } state abort
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR) || coil_rbv != last_coil) {
            strcpy(reason, "relay moved or lost");
        } state abort
        when (now_s() - t0 > zero_timeout) {
            sprintf(reason, "I at off: %.3g A", i_meas);
        } state abort
        when (since >= 0 && fabs(i_meas) < threshold && now_s() - since >= hold) {
            /* A fresh read of what the coil write depends on, stamped within 2.5 s. */
            stale = 0;
            if (pvGet(i_meas, SYNC, 2.0) != pvStatOK || pvGet(out_rbv, SYNC, 2.0) != pvStatOK) {
                stale = 1;
            } else {
                age_i = age_of(pvTimeStamp(i_meas));
                age_o = age_of(pvTimeStamp(out_rbv));
                if (age_i > 2.5 || age_o > 2.5 || pvSeverity(i_meas) >= pvSevrMAJOR
                    || pvSeverity(out_rbv) >= pvSevrMAJOR || fabs(i_meas) >= threshold || out_rbv != 0) {
                    stale = 1;
                }
            }
            ok = 1;
            if (!stale) {
                step = 4;  pvPut(step, SYNC);
                coil_sp = want_coil;
                ok = (pvPut(coil_sp, SYNC) == pvStatOK);
            }
            t0 = now_s();
        } state switching
        when (delay(0.1)) {
            if (fabs(i_meas) < threshold) {
                if (since < 0) since = now_s();
            } else {
                since = -1;
            }
        } state checking
    }

    state switching {
        when (stale) { strcpy(reason, "readback stale at coil"); } state abort
        when (!ok) { strcpy(reason, "coil write failed"); } state abort
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR)) {
            strcpy(reason, "relay readback lost");
        } state abort
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR) || out_rbv != 0) {
            strcpy(reason, "output not off");
        } state abort
        when (coil_rbv == want_coil) {
            last_coil = coil_rbv;
            step = 8;  pvPut(step, SYNC);
            t0 = now_s();
        } state settling
        when (now_s() - t0 > coil_timeout) { strcpy(reason, "coil did not read back"); } state abort
        when (delay(0.1)) {} state switching
    }

    state settling {
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR) || coil_rbv != want_coil) {
            strcpy(reason, "relay moved or lost");
        } state abort
        when (!(pvConnected(i_meas) && pvSeverity(i_meas) < pvSevrMAJOR)) {
            strcpy(reason, "current readback lost");
        } state abort
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR) || out_rbv != 0) {
            strcpy(reason, "output not off");
        } state abort
        when (now_s() - t0 >= settle && was_on
              && !(pvConnected(fault_rbv) && pvSeverity(fault_rbv) < pvSevrINVALID && fault_rbv == 0
                   && pvConnected(panel_rbv) && pvSeverity(panel_rbv) < pvSevrINVALID && panel_rbv == 0
                   && pvConnected(ilk_rbv) && pvSeverity(ilk_rbv) < pvSevrINVALID && ilk_rbv == 0)) {
            strcpy(reason, "unit fault/panel/ilk");
        } state abort
        when (now_s() - t0 >= settle) {
            ok = 1;
            if (was_on) {
                step = 5;  pvPut(step, SYNC);
                out_sp = 1;
                ok = (pvPut(out_sp, SYNC) == pvStatOK);
            }
            t0 = now_s();
        } state output_on
        when (delay(0.1)) {} state settling
    }

    state output_on {
        when (!ok) { strcpy(reason, "output on write failed"); } state abort
        when (!(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR)) {
            strcpy(reason, "output readback lost");
        } state abort
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR) || coil_rbv != want_coil) {
            strcpy(reason, "relay moved or lost");
        } state abort
        when (out_rbv == was_on) {
            step = 6;  pvPut(step, SYNC);
            sp_raw = fabs(target);
            ok = (pvPut(sp_raw, SYNC) == pvStatOK);
            t0 = now_s();
        } state setting
        when (now_s() - t0 > out_timeout) { strcpy(reason, "output did not read on"); } state abort
        when (delay(0.1)) {} state output_on
    }

    state setting {
        when (!ok) { strcpy(reason, "write of |I| failed"); } state abort
        when (!(pvConnected(coil_rbv) && pvSeverity(coil_rbv) < pvSevrMAJOR) || coil_rbv != want_coil) {
            strcpy(reason, "relay moved or lost");
        } state abort
        when (was_on && !(pvConnected(out_rbv) && pvSeverity(out_rbv) < pvSevrMAJOR && out_rbv == 1)) {
            strcpy(reason, "output off by another");
        } state abort
        when (fabs(sp_rbv - fabs(target)) < 0.0005 && pvConnected(sp_rbv) && pvSeverity(sp_rbv) < pvSevrMAJOR) {
            sprintf(msg, "polarity %s, %.3f A%s", target < 0 ? "neg" : "pos", target, was_on ? "" : ", off");
            pvPut(msg, SYNC);
            efClear(sp_ev); efClear(out_ev);       /* a put that slipped in during the sequence is dropped */
            step = 0;  pvPut(step, SYNC);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
        when (now_s() - t0 > out_timeout) { strcpy(reason, "unit did not take |I|"); } state abort
        when (delay(0.1)) {} state setting
    }

    state abort {
        entry {
            efClear(reset_ev);                     /* only a reset after this abort clears it */
            out_sp = 0;
            pvPut(out_sp, SYNC);
            off_confirmed = 0;
            retry_off = 1;
            sprintf(msg, "ABORT %.22s", reason); pvPut(msg, SYNC);
            step = 7;  pvPut(step, SYNC);
            busy = 1;  pvPut(busy, SYNC);          /* puts stay refused until reset */
        }
        when () {} state aborted
    }
}
