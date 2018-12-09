# PPMOutput
Arduino RC app for 2-wire interface to MEGA2560, with up to 6 RC servos using JRPROPO R700 Rx and optionally Max66ADT (XP662) Tx.

All servos are always connected to the JRPROPO RX (e.g. R700), but the servo PPM command stream can be taken from 1 of 4 configurations:
CFG 1 - the Arduino hardwired to the RC Rx, or
CFG 2 - manual control by the RC Tx while hardwired to the RC Tx, or
CFG 3 - the Arduino hardwired to the RC Tx, and then transmitted by RF to the RC Rx, or
CFG 4 - the trivial case where the Arduino is not used, and manaual control by the RC Tx is transmitted by RF to the RC Rx.

