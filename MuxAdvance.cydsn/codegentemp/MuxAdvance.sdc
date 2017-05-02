# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Users\Lenovo\Desktop\Hardware\Software Mux\Mux Advance Encore\MuxAdvance.cydsn\MuxAdvance.cyprj
# Date: Wed, 26 Apr 2017 19:20:42 GMT
#set_units -time ns
create_clock -name {CyILO} -period 1000000 -waveform {0 500000} [list [get_pins {ClockBlock/ilo}] [get_pins {ClockBlock/clk_100k}] [get_pins {ClockBlock/clk_1k}] [get_pins {ClockBlock/clk_32k}]]
create_clock -name {CyIMO} -period 333.33333333333331 -waveform {0 166.666666666667} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyPLL_OUT} -period 16.666666666666664 -waveform {0 8.33333333333333} [list [get_pins {ClockBlock/pllout}]]
create_clock -name {CyMASTER_CLK} -period 16.666666666666664 -waveform {0 8.33333333333333} [list [get_pins {ClockBlock/clk_sync}]]
create_generated_clock -name {CyBUS_CLK} -source [get_pins {ClockBlock/clk_sync}] -edges {1 2 3} [list [get_pins {ClockBlock/clk_bus_glb}]]
create_clock -name {CyBUS_CLK(fixed-function)} -period 16.666666666666664 -waveform {0 8.33333333333333} [get_pins {ClockBlock/clk_bus_glb_ff}]
create_generated_clock -name {Beagle_IntClock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 65 131} [list [get_pins {ClockBlock/dclk_glb_0}]]
create_generated_clock -name {LCD1_IntClock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 65 131} [list [get_pins {ClockBlock/dclk_glb_1}]]
create_generated_clock -name {Tag_IntClock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 65 131} [list [get_pins {ClockBlock/dclk_glb_2}]]
create_generated_clock -name {LCD2_IntClock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 65 131} [list [get_pins {ClockBlock/dclk_glb_3}]]
create_generated_clock -name {Code_Bar_IntClock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 195 391} [list [get_pins {ClockBlock/dclk_glb_4}]]
create_generated_clock -name {Clock_1} -source [get_pins {ClockBlock/clk_sync}] -edges {1 1305 2609} [list [get_pins {ClockBlock/dclk_glb_5}]]
create_generated_clock -name {timer_clock_1} -source [get_pins {ClockBlock/clk_sync}] -edges {1 60001 120001} [list [get_pins {ClockBlock/dclk_glb_6}]]
create_clock -name {timer_clock_1(fixed-function)} -period 1000000 -waveform {0 16.6666666666667} [get_pins {ClockBlock/dclk_glb_ff_6}]

set_false_path -from [get_pins {__ONE__/q}]

# Component constraints for C:\Users\Lenovo\Desktop\Hardware\Software Mux\Mux Advance Encore\MuxAdvance.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Users\Lenovo\Desktop\Hardware\Software Mux\Mux Advance Encore\MuxAdvance.cydsn\MuxAdvance.cyprj
# Date: Wed, 26 Apr 2017 19:19:59 GMT
