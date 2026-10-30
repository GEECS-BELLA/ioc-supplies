/* emqComposite: the polarity sequence of one bipolar EMQ magnet (U_EMQTripletBipolar).
 *
 * One instance per magnet, after iocInit:
 *   seq emqComposite, "P=undulator:emqtest1:,CH=ch1,SUP=undulator:emqtest1_ps:,PLC=undulator:plctest1:,COIL=15"
 * The composite's records (geecs-epics services/undulator/supplies/emqtest1/config/
 * emq_composite.template) hold the signed setpoint, busy, state, message, reset and the named
 * values. This program only sequences. A put to the setpoint is refused by access security
 * while busy_CH reads 1 (the setpoint's ASG reads it), so the program sees no put during a
 * sequence or while latched.
 *
 * A put with the relay's present polarity, or 0 A: the unit's current is set directly. A put
 * with the other sign:
 *   0 A -> |I| < threshold held for hold s (zero timeout)
 *       -> output off, read back (output timeout)
 *       -> |I| < threshold held again with the output off (zero timeout)
 *       -> coil write, coil readback (coil timeout)
 *       -> output on, read back (output timeout)
 *       -> |value|.
 * Any failed write, timeout or mismatch: output off, state "aborted" (MAJOR: the mbbo's state
 * severity), busy 1, latched until reset_CH. At start every channel is latched ("restarted"):
 * a restart mid-sequence resumes nothing, and nothing is written at start.
 *
 * Timeouts are measured with the clock (now_s), not delay(): a self-transition restarts delay().
 */
program emqComposite("P=,CH=,SUP=,PLC=,COIL=")

option +r;      /* reentrant: one instance per magnet */
option -c;      /* start without waiting for every channel to connect */

%%#include <math.h>
%%#include <stdio.h>
%%#include <string.h>
%%#include <epicsTime.h>

/* The composite's own records */
double sp;            assign sp           to "{P}current_limit_{CH}:SP";  monitor sp;
evflag sp_ev;         sync sp sp_ev;
short  busy;          assign busy         to "{P}busy_{CH}";
short  step;          assign step         to "{P}state_{CH}";
string msg;           assign msg          to "{P}message_{CH}";
short  reset;         assign reset        to "{P}reset_{CH}";             monitor reset;
short  neg_coil;      assign neg_coil     to "{P}config:negative_when_coil_{CH}"; monitor neg_coil;
double threshold;     assign threshold    to "{P}config:zero_threshold_a"; monitor threshold;
double hold;          assign hold         to "{P}config:zero_hold_s";      monitor hold;
double zero_timeout;  assign zero_timeout to "{P}config:zero_timeout_s";   monitor zero_timeout;
double out_timeout;   assign out_timeout  to "{P}config:output_timeout_s"; monitor out_timeout;
double coil_timeout;  assign coil_timeout to "{P}config:coil_timeout_s";   monitor coil_timeout;

/* The unit (emqtest1-ps) and the relay (plctest1) */
double sp_raw;        assign sp_raw       to "{SUP}current_limit_{CH}:SP";
double i_meas;        assign i_meas       to "{SUP}current_{CH}";         monitor i_meas;
short  out_sp;        assign out_sp       to "{SUP}enable_output_{CH}:SP";
short  out_rbv;       assign out_rbv      to "{SUP}enable_output_{CH}";   monitor out_rbv;
short  coil_sp;       assign coil_sp      to "{PLC}do_ch{COIL}:SP";
short  coil_rbv;      assign coil_rbv     to "{PLC}do_ch{COIL}";          monitor coil_rbv;

/* step (state_CH) values, the mbbo's states in order: 0 idle, 1 zeroing, 2 output_off,
   3 checking, 4 switching, 5 output_on, 6 setting, 7 aborted */

double target;        /* the signed value being set */
short  want_coil;
int    ok;            /* the last write's outcome */
double t0;            /* when this step began */
double since;         /* when |I| first read under the threshold in this hold; < 0: not yet */
char   reason[40];

%{
static int sign_of(short coil, short neg) { return coil == neg ? -1 : 1; }
static double now_s(void)
{
    epicsTimeStamp t;
    epicsTimeGetCurrent(&t);
    return t.secPastEpoch + t.nsec * 1e-9;
}
}%

ss polarity {
    state start {
        entry {
            busy = 1;  pvPut(busy, SYNC);
            step = 7;  pvPut(step, SYNC);
            strcpy(msg, "restarted: reset to resume"); pvPut(msg, SYNC);
        }
        when () {} state aborted
    }

    state aborted {
        when (reset) {
            reset = 0; pvPut(reset, SYNC);
            efClear(sp_ev);
            step = 0;  pvPut(step, SYNC);
            busy = 0;  pvPut(busy, SYNC);
            strcpy(msg, "reset"); pvPut(msg, SYNC);
        } state idle
    }

    state idle {
        when (reset) {                  /* a reset with nothing to clear: consumed, so it never */
            reset = 0; pvPut(reset, SYNC);   /* clears a later abort by itself */
        } state idle
        when (efTestAndClear(sp_ev)) {
            target = sp;
        } state decide
    }

    state decide {
        when (!pvConnected(coil_rbv) || !pvConnected(out_rbv) || !pvConnected(i_meas)) {
            strcpy(reason, "a link is disconnected");
        } state abort
        when (target == 0.0 || (target > 0) == (sign_of(coil_rbv, neg_coil) > 0)) {
            sp_raw = fabs(target);
            if (pvPut(sp_raw, SYNC) == pvStatOK) {
                sprintf(msg, "set %.3f A", target);
            } else {
                sprintf(msg, "unit refused %.3f A", fabs(target));
            }
            pvPut(msg, SYNC);
        } state idle
        when () {
            busy = 1;  pvPut(busy, SYNC);
            want_coil = (target < 0) ? neg_coil : 1 - neg_coil;
            step = 1;  pvPut(step, SYNC);
            sp_raw = 0.0;
            ok = (pvPut(sp_raw, SYNC) == pvStatOK);
            t0 = now_s();
            since = -1;
        } state zeroing
    }

    state zeroing {
        when (!ok) {
            strcpy(reason, "write 0 A failed");
        } state abort
        when (now_s() - t0 > zero_timeout) {
            sprintf(reason, "I did not fall: %.3g A", i_meas);
        } state abort
        when (since >= 0 && fabs(i_meas) < threshold && now_s() - since >= hold) {
            step = 2;  pvPut(step, SYNC);
            out_sp = 0;
            ok = (pvPut(out_sp, SYNC) == pvStatOK);
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
        when (!ok) {
            strcpy(reason, "output off write failed");
        } state abort
        when (out_rbv == 0) {
            step = 3;  pvPut(step, SYNC);
            t0 = now_s();
            since = -1;
        } state checking
        when (now_s() - t0 > out_timeout) {
            strcpy(reason, "output did not read off");
        } state abort
        when (delay(0.1)) {} state output_off
    }

    state checking {
        when (now_s() - t0 > zero_timeout) {
            sprintf(reason, "I with output off: %.3g A", i_meas);
        } state abort
        when (since >= 0 && fabs(i_meas) < threshold && now_s() - since >= hold) {
            step = 4;  pvPut(step, SYNC);
            coil_sp = want_coil;
            ok = (pvPut(coil_sp, SYNC) == pvStatOK);
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
        when (!ok) {
            strcpy(reason, "coil write failed");
        } state abort
        when (coil_rbv == want_coil) {
            step = 5;  pvPut(step, SYNC);
            out_sp = 1;
            ok = (pvPut(out_sp, SYNC) == pvStatOK);
            t0 = now_s();
        } state output_on
        when (now_s() - t0 > coil_timeout) {
            strcpy(reason, "coil did not read back");
        } state abort
        when (delay(0.1)) {} state switching
    }

    state output_on {
        when (!ok) {
            strcpy(reason, "output on write failed");
        } state abort
        when (out_rbv == 1) {
            step = 6;  pvPut(step, SYNC);
            sp_raw = fabs(target);
            ok = (pvPut(sp_raw, SYNC) == pvStatOK);
        } state setting
        when (now_s() - t0 > out_timeout) {
            strcpy(reason, "output did not read on");
        } state abort
        when (delay(0.1)) {} state output_on
    }

    state setting {
        when (!ok) {
            strcpy(reason, "write of |I| failed");
        } state abort
        when () {
            sprintf(msg, "polarity %s, %.3f A", target < 0 ? "neg" : "pos", target);
            pvPut(msg, SYNC);
            step = 0;  pvPut(step, SYNC);
            busy = 0;  pvPut(busy, SYNC);
        } state idle
    }

    state abort {
        entry {
            out_sp = 0;
            if (pvPut(out_sp, SYNC) == pvStatOK) {
                sprintf(msg, "ABORT %.33s", reason);
            } else {
                sprintf(msg, "ABORT, OFF FAILED %.21s", reason);
            }
            pvPut(msg, SYNC);
            step = 7;  pvPut(step, SYNC);
            busy = 1;  pvPut(busy, SYNC);    /* puts stay refused until reset */
        }
        when () {} state aborted
    }
}
