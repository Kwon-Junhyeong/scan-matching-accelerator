# Creates the CORDIC core used by stage 1 (settings of the board design).
#   vivado -mode batch -source ip/cordic_sin_cos.tcl -tclargs <output dir> [part]
# The board design used xczu7ev-ffvc1156-2-e (ZCU104).

set out  [file normalize [lindex $argv 0]]
set part [expr {[llength $argv] > 1 ? [lindex $argv 1] : "xczu7ev-ffvc1156-2-e"}]

file mkdir $out
create_project -in_memory -part $part
create_ip -name cordic -vendor xilinx.com -library ip -module_name cordic_sin_cos -dir $out
set_property -dict [list \
    CONFIG.Functional_Selection        {Sin_and_Cos} \
    CONFIG.Architectural_Configuration {Parallel} \
    CONFIG.Pipelining_Mode             {Maximum} \
    CONFIG.Data_Format                 {SignedFraction} \
    CONFIG.Phase_Format                {Scaled_Radians} \
    CONFIG.Input_Width                 {16} \
    CONFIG.Output_Width                {16} \
    CONFIG.Round_Mode                  {Truncate} \
    CONFIG.Iterations                  {0} \
    CONFIG.Precision                   {0} \
    CONFIG.Coarse_Rotation             {true} \
    CONFIG.Compensation_Scaling        {No_Scale_Compensation} \
    CONFIG.flow_control                {Blocking} \
    CONFIG.out_tready                  {true} \
    CONFIG.ACLKEN                      {false} \
    CONFIG.ARESETN                     {true} \
] [get_ips cordic_sin_cos]
generate_target {synthesis} [get_ips cordic_sin_cos]
