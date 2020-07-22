case $1 in
    synth)  # synth
        iverilog -g 2005 -o pipeline pipeline.v
        ;;
    run)    # run
        vvp pipeline
        ;;
    wave)   # wave
        gtkwave dump.vcd
        ;;
esac
