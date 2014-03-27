create_clock -name "Clock50" -period 20.000ns [get_ports {CLOCK_50}]

derive_pll_clocks -create_base_clocks

set_input_delay -clock "Clock50" -max -20ns [all_inputs] 
set_input_delay -clock "Clock50" -min 0ns [all_inputs] 

set_output_delay -clock "Clock50" -max -20ns [all_outputs] 
set_output_delay -clock "Clock50" -min 0ns [all_outputs] 
