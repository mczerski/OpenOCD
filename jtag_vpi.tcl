source [find ./tcl/interface/jtag_vpi.cfg]

jtag_vpi_set_port 50020

source [find ./tcl/target/or1k.cfg]

gdb_port 50002

if { [catch { init } ] } {
	puts "Couldn't connect to JTAG server..."
	shutdown
} else {

	echo "Halting processor"
	halt

	foreach name [target names] {
	    set y [$name cget -endian]
	    set z [$name cget -type]
	    puts [format "Chip is %s, Endian: %s, type: %s" \
			  $name $y $z]
	}

}