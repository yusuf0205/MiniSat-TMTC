/*
 * icd.h - MiniSat TM/TC Interface Control Document, as code.
 *
 * This is the SINGLE source of truth for every number shared by the
 * flight software and the ground station (ground/icd.py mirrors it and
 * tests/test_icd_sync.py checks the two never drift apart).
 *
 * All values are invented for this project. No L&T / LTSAT data.
 */
#ifndef ICD_H
#define ICD_H

#include <stdint.h>

/* ---------------- Link layer (HDLC / AX.25-style) ---------------- */
#define HDLC_FLAG            0x7E
#define AX25_ADDR_LEN        7
#define AX25_CTRL_UI         0x03      /* unnumbered information frame   */
#define AX25_PID_NONE        0xF0      /* no layer-3 protocol            */
#define AX25_HDR_LEN         (2 * AX25_ADDR_LEN + 2)
#define FRAME_FCS_LEN        2
#define LINK_BITRATE_BPS     9600      /* both directions                */

/* ---------------- CCSDS space packet ---------------- */
#define CCSDS_PRI_HDR_LEN    6
#define CCSDS_TYPE_TM        0
#define CCSDS_TYPE_TC        1
#define CCSDS_SEQ_STANDALONE 3         /* grouping flags '11'            */
#define CCSDS_SEQ_MOD        16384
#define CUC_PFIELD           0x2E      /* 4 B coarse + 2 B fine, agency epoch */
#define CUC_LEN              7         /* P-field + 4 + 2                */
#define TC_TIMETAG_LEN       8         /* CUC (7) + 1 reserved byte       */
#define TM_SEC_HDR_LEN       CUC_LEN   /* OBC time on every TM packet     */
#define PKT_CRC_LEN          2
#define MAX_PACKET_LEN       224
#define MAX_TM_SOURCE_LEN    200
#define APID_IDLE            0x7FF

/* TC APIDs: one per destination */
#define APID_TC_OBC          0x100
#define APID_TC_ADCS         0x110
#define APID_TC_EPS          0x120
#define APID_TC_COMMS        0x130

/* TM APIDs: one per kind of telemetry */
#define APID_TM_HK           0x200     /* real-time housekeeping (default) */
#define APID_TM_ACK          0x201     /* TC verification: acceptance/execution */
#define APID_TM_ONREQ        0x202     /* answer to a GET                  */
#define APID_TM_STORED_HK    0x203     /* housekeeping played back by DUMP */
#define APID_TM_SSTM         0x204     /* super-sampled ADCS data          */
#define APID_TM_EVENT        0x205     /* FDIR / event reports             */

/* ---------------- Command IDs ---------------- */
/* OBC */
#define CMD_NO_OP            0x0001
#define CMD_DUMP_STM         0x0003   /* u16 max_packets                   */
#define CMD_SET_HK_PERIOD    0x0004   /* u8 seconds (1..60)                */
#define CMD_TLE_UPLOAD       0x0005   /* 2 x 69 ASCII chars                */
#define CMD_CLEAR_SCHEDULE   0x0006
#define CMD_SET_MODE         0x0007   /* u8 0=SAFE 1=NOMINAL               */
/* ADCS */
#define CMD_ADCS_GET_PARAM   0x0010   /* u8 param id -> ONREQ TM            */
#define CMD_ADCS_SET_WHEEL   0x0011   /* i16 rpm (-6000..6000)              */
#define CMD_ADCS_SSTM_START  0x0012   /* u8 rate Hz (1..50), u32 duration ms (100..60000) */
#define CMD_ADCS_MTQ_ONOFF   0x0013   /* u8 0/1                             */
/* EPS */
#define CMD_EPS_SET_RAIL     0x0020   /* u8 rail (1=payload,2=wheel), u8 on */
#define CMD_EPS_SET_SAFE_MV  0x0021   /* u16 mV (6200..7400)                */
/* COMMS */
#define CMD_COMMS_TX_POWER   0x0030   /* u8 dBm (20..33)                    */

/* GET parameter ids */
#define PARAM_WHEEL_RPM      1
#define PARAM_BATT_MV        2
#define PARAM_MAG_X_NT       3
#define PARAM_TLE_EPOCH      4
#define PARAM_TX_POWER       5
#define PARAM_SAFE_MV        6

/* ---------------- Verification (ACK) packets ---------------- */
#define ACK_ACCEPT_OK        1
#define ACK_ACCEPT_FAIL      2
#define ACK_EXEC_OK          3
#define ACK_EXEC_FAIL        4

#define ERR_NONE             0
#define ERR_BAD_APID         2
#define ERR_DUPLICATE        3
#define ERR_MODE             4
#define ERR_BAD_CMD          5
#define ERR_BAD_PARAM        6
#define ERR_SCHED_FULL       7
#define ERR_EXEC             8
#define ERR_ABORTED_SAFE     9
#define ERR_TIME_PAST        10
#define ERR_BAD_LENGTH       11

/* ---------------- Events ---------------- */
#define EVT_SAFE_ENTER       1
#define EVT_SAFE_EXIT        2
#define EVT_SSTM_DONE        3
#define EVT_TLE_UPDATED      4
#define EVT_STM_OVERFLOW     5

/* ---------------- Modes ---------------- */
#define MODE_SAFE            0
#define MODE_NOMINAL         1

#endif
