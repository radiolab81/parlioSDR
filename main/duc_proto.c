/* duc_proto.c - siehe duc_proto.h */
#include "duc_proto.h"
#include <string.h>
#include <math.h>

void duc_proto_init(duc_proto_t *p)
{
    memset(p, 0, sizeof *p);
}

double duc_ftw_to_hz_signed(uint32_t ftw)
{
    /* Der Streamer wandelt negative Frequenzen per static_cast<uint32_t> in einen
     * Zweierkomplement-FTW; der NCO rechnet ohnehin modulo 2^32. */
    return (double)(int32_t)ftw / 4294967296.0 * DUC_FPGA_REF_HZ;
}

uint32_t duc_ftw_to_rate_hz(uint32_t ftw)
{
    /* Auf ganze Hertz runden: der FTW-Rundungsfehler betraegt < 0,01 Hz. */
    double hz = (double)ftw / 4294967296.0 * DUC_FPGA_REF_HZ;
    return (uint32_t)llround(hz);
}

static bool make_event(duc_proto_t *p, duc_event_t *ev)
{
    /* Execute: vollstaendigen Befehl in ein Ereignis umsetzen */
    memset(ev, 0, sizeof *ev);
    if (!p->cmd_valid) {
        ev->type = DUC_EV_BAD;
        p->n_bad++;
        return true;
    }
    ev->cmd = p->cmd;
    if ((p->cmd == 'S' || p->cmd == 'R') && p->data_mask != 0x0F) {
        ev->type = DUC_EV_BAD;                 /* FTW unvollstaendig */
        p->n_bad++;
    } else {
        ev->value = (uint32_t)p->data[0] | ((uint32_t)p->data[1] << 8) |
                    ((uint32_t)p->data[2] << 16) | ((uint32_t)p->data[3] << 24);
        ev->type = p->cmd == 'S' ? DUC_EV_SHIFT : p->cmd == 'R' ? DUC_EV_RATE : DUC_EV_UNKNOWN;
        p->n_cmds++;
    }
    p->cmd_valid = false;                      /* jeder Befehl braucht einen eigenen Beginn */
    return true;
}

duc_feed_t duc_proto_feed(duc_proto_t *p, const uint8_t *in, size_t n, size_t *used,
                          int16_t *iq, size_t iq_cap, size_t *n_iq, duc_event_t *ev)
{
    size_t u = 0, cnt = 0;
    duc_feed_t ret = DUC_FEED_DONE;

    while (u < n) {
        /* Platz fuer ein weiteres Paar sicherstellen, bevor ein Wort verbraucht wird */
        if (cnt >= iq_cap) { ret = DUC_FEED_FULL; break; }

        if (!p->have_lo) { p->lo = in[u++]; p->have_lo = true; continue; }
        const uint16_t w = (uint16_t)(p->lo | ((uint16_t)in[u++] << 8));
        p->have_lo = false;

        switch (w >> 14) {
        case DUC_TAG_I:
            if (p->have_i) p->drop_i++;                    /* vorheriges I blieb ohne Q */
            p->i_val = (int16_t)(uint16_t)(w << 2);        /* 14 Bit -> 16 Bit, Vorzeichen bleibt */
            p->have_i = true;
            break;
        case DUC_TAG_Q:
            if (!p->have_i) { p->drop_q++; break; }
            iq[2 * cnt]     = p->i_val;
            iq[2 * cnt + 1] = (int16_t)(uint16_t)(w << 2);
            cnt++;
            p->n_pairs++;
            p->have_i = false;
            break;
        case DUC_TAG_CMD: {
            const unsigned idx = (w >> 8) & 0x3F;
            const uint8_t  pay = (uint8_t)(w & 0xFF);
            if (idx == DUC_CMD_BEGIN_INDEX) {
                p->cmd_valid = true; p->cmd = pay;
                memset(p->data, 0, sizeof p->data); p->data_mask = 0;
            } else if (idx < 4 && p->cmd_valid) {
                p->data[idx] = pay; p->data_mask |= (uint8_t)(1u << idx);
            }                                              /* alles andere: ignorieren */
            break;
        }
        default: /* DUC_TAG_EXEC */
            if ((w & 0xFF) == 'E') {
                make_event(p, ev);
                *used = u; *n_iq = cnt;
                return DUC_FEED_EVENT;
            }
            break;
        }
    }
    *used = u; *n_iq = cnt;
    return ret;
}
