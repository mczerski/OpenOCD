openOCD
=======

openOCD including OpenRISC support with Mohor debug interface and
Nathan Yawn adv_dbg_if debug interface.

A JTAG driver acting as a client for a VPI JTAG server interface is also
included.

    +---------------     +----------------+      +----------------+      +------------------+      +----------+ 
    +              +     +                +      +                +      +                  +      +          + 
    + OpenOCD core + --> + JAG VIP driver + <==> + JAG VIP server + <--> + JTAG VPI verilog + <--> + JTAG TAP + 
    +              +     +                +      +                +      +                  +      +          + 
    +---------------     +----------------+      +----------------+      +------------------+      +----------+
                             jtag_vpi.c              jtag_vpi.c               jtag_vpi.v             any tap...
    --------------------------------------- TCP  ------------------  VPI --------------------------------------
    ---------------------------------------      --------------------------------------------------------------
                  OpenOCD                                                VPI + Verilog RTL

If you want to use the VPI JTAG driver, you need to configure OpenOCD like this:

./configure --enable-jtag_vpi --enable-adv_debug_sys --enable-maintainer-mode



